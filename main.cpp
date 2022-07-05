#include "mbed.h"
#include "SDFileSystem.h"  
#include "rtos.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "ThirdOctave.h"
#include "NMEA.h"
#include "PondFilter.h" 
#include "string" 
#include <ctype.h>
#include <cstdlib>
#include "main.h"

#define mic 1
Semaphore holdS(0);
Mutex resS;
int tmpAux=0;

void errorISR()
{
    led=!led;
}

void update(void const *args) {
    //pc.printf("updateId: %x\r\n",osThreadGetId ());
    while(true)
    {
        semSave.wait();
        fclose(filePtr); 
        filePtr = fopen(fileName,"a");
        if(filePtr==NULL) 
        {
            printf("Error writing log\r\n");
            stopSampling();
        }
    }
}

void getGPSData(void const *args) {
    //pc.printf("gpsId: %x\r\n",osThreadGetId ()); 
    Thread::signal_wait(0x1);   
    while(true)
    {        
        semGPS.wait();
        if(gps & start)
            GPSmodule.sampleGPS();  
    }
}

void ticker125ISR()
{    
    sem125.release();
}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *MEMSmic)
{   
    HAL_I2S_Receive_DMA(MEMSmic,&samplesU16[0],N*2);  
    tmpAux++;    
}

void computations(void const *args)
{
    //pc.printf("ComputationsId: %x\r\n",osThreadGetId ());
    while(true)
    {                        
        
        float Pxx = 0.0;    //Calculate power
        double Pxf = 0.0;   //Power in dB
        float auxF=0;                
        
        sem125.wait();  
              
        printf("\r\ntmpAux:%d",tmpAux);
        
        for(int i=0; i<N; i++) 
        {            
            //pc.printf("Received0[%d]: %x - %x - %x - %x\r\n",i,samplesU16[i*4],samplesU16[i*4+1],samplesU16[i*4+2],samplesU16[i*4+3]);                        
            auxU162[0] = 0;
            auxU162[1] = 0;
            
            switch(mic)
            {
                case 0:
                    for(int j=0; j<4; j++) 
                    {
                        if((samplesU16[i*4+j] & 0x1FFE)!=0) 
                            auxU162[0] = (samplesU16[i*4+j]& 0xFFFF);  //Verify change from 0x7FFF to 0xFFFF  
                        else if(samplesU16[i*4+j]!=0)
                            auxU162[1] = samplesU16[i*4+j]; 
                    } 
                                           
                    auxF = (float)(( (auxU162[0]<<2) | (auxU162[1]>>14 & 0x0003) ) & 0x0003FFFF);
                    if(auxF>65536)
                        auxSamples[i] = (auxF - 65536);
                    else
                        auxSamples[i] = (65536 + auxF);
                break;
                
                case 1:
                    for(int j=0; j<4; j++) 
                    {
                        if((samplesU16[i*4+j] & 0x007F)!=0) 
                            auxU162[0] = samplesU16[i*4+j] & 0x7FFF;    
                        else if(samplesU16[i*4+j]!=0)
                            auxU162[1] = samplesU16[i*4+j] & 0xFF80;
                    } 
                                           
                    auxF = (((auxU162[0]<<9 & 0x00FFFE00) | (auxU162[1]>>7 & 0x000001FF)) & 0x00FFFFFF);
                    if(auxF>8388608)
                        auxSamples[i] = (auxF - 8388608);
                    else
                        auxSamples[i] = (8388608 + auxF);
                break;
            }
            //pc.printf("%f\r\n",auxSamples[i]);                                                                    
            //pc.printf("Received1[%d]: %x - %x\r\n",i,auxU162[0],auxU162[1]);
            //pc.printf("%f\r\n",auxU162[0]);
            //auxSamples[i]=sin(2*pi*i*f/Fs);                                                          
        }
        //stopSampling(); 
        
        //Remove DC component in time domain
        arm_mean_f32(auxSamples, N, &auxF);         
        for(int i=0;i<N;i++)
            auxSamples[i]=auxSamples[i]-auxF;
            
        arm_biquad_cascade_df2T_init_f32(&iirS, NUM_SECTIONS, AFilterCoeff, iir_state);    
        if(PondFilter=='A')
            arm_biquad_cascade_df2T_f32Mod(&iirS, auxSamples, outFFT, N, AFilterGains); // A Filter                              
        arm_rms_f32(outFFT,N,&Pxx);
        Pxf = 20*log10(Pxx)+cal;
        LeqFastSamples[LeqFastCount]=Pxf;
        LeqFastCount++;
        
        if(LeqFastCount>=8) 
        {
            tickADC125.detach();
            LeqFastCount=0;
            LeqFastFull=true;

            LeqSeconds[LeqSecondsCount]=sumaLeq(LeqFastSamples,NLeqSlow);   //Compute 1s Leq and save it in global seconds var
            LeqSecondsCount++;  //Increase 1s sampling count
            Th1sData->signal_set(0x1);
        }
            
        if(octave) 
        {                        
            int initIRR = 5*NUM_SECTIONS;
            for(int ss=0;ss<NFilters;ss++)
            {                                
                arm_biquad_cascade_df2T_init_f32(&iirS, NUM_SECTIONS, &thirdOctIIR[ss*initIRR], iir_state);    
                arm_biquad_cascade_df2T_f32Mod(&iirS, auxSamples, outFFT, N,&gains[ss*3]); //Filter
                arm_rms_f32(outFFT,N,&Pxx);
                LeqFastTOB[ss][LeqFastTOBCount] = 20*log10(Pxx) + cal;// + calTOB[ss];
            }                       
            
            LeqFastTOBCount++;
            if(LeqFastTOBCount>=8)  //Compute 1s samples
            {
                LeqFastTOBCount=0;        
                for(int i=0;i<NFilters;i++)
                    LeqSecondsTOB[i][LeqSecondsCount-1]=sumaLeq(LeqFastTOB[i],NLeqSlow);                                       
            }                        
        }
    }    
}

bool insertHeader()
{    
    filePtr = fopen(fileName,"w");
    string data;
    if(filePtr) 
    {
        data.append("Leq\t\t|\tDate/Time\t\t\t");
        if(gps)
            data.append("\t|\tLAT\t\t\t|\tLON\t\t\t|\tQ\t|SAT#");
        if(octave)
            data.append("\t|TOB_24.803\t|TOB_31.250\t|TOB_39.373\t|TOB_49.606\t|TOB_62.500\t|TOB_78.745\t|TOB_99.213\t|TOB_125.00\t|TOB_157.49\t|TOB_198.43\t|TOB_250.00\t|TOB_314.98\t|TOB_396.85\t|TOB_500.00\t|TOB_629.96\t|TOB_793.70\t|TOB_1000.0\t|TOB_1259.9\t|TOB_1587.4\t|TOB_2000.0\t|TOB_2519.8\t|TOB_3174.8\t|TOB_4000.0\t|TOB_5039.7\t|TOB_6349.6\t|TOB_8000.0\t|TOB_10079.3");
        data.append("\r\n\r\n");
        //printf("Header: %s",data.c_str());    
        fprintf(filePtr,"%s",data.c_str());
        fclose(filePtr);  
        return true;     
    }else
        printf("Error writing header\r\n");
    return false;  
} 

void saveData(float LeqVal, double *TOBdata)
{
    char currTimeBuff[32];
    char tmp[200];
    time_t seconds =  rtc_read();
    strftime(currTimeBuff, 32, "%d/%m/%y %T", localtime(&seconds));

    sprintf(tmp,"%04.1f\t|\t%s\t",LeqVal,currTimeBuff);
    fprintf(filePtr,"%s",tmp);
    if(gps) {
        sprintf(tmp,"\t|\t%f\t|\t%f\t|\t%d\t|\t%d",GPSmodule.getLat(),GPSmodule.getLon(),GPSmodule.getFixQuality(),GPSmodule.getSatellites());
        fprintf(filePtr,"%s",tmp);
    }
    if(octave) {
        for(int i=0; i<NFilters; i++) {
            sprintf(tmp,"\t|\t%04.1f",TOBdata[i]);
            fprintf(filePtr,"%s",tmp);
        }
    }
    fprintf(filePtr,"\r\n");

    
    if(RecTime!=0 && seconds>=endTime)
        stopSampling();   
    if(auxSave>limit)
    {
        auxSave=0;
        semSave.release(); 
    }
    
    //Debug
    currTime=t.read_ms();
    rel=1.0f*currTime/tcount;
    pc.printf("Leq: %04.1f, T[%d]=%d\t%s\r\n",sumaLeq(LeqSeconds,LeqTime),tcount,currTime-prevTime, currTimeBuff);
    int i=100;
    //pc.printf("Received0[%d]: %x - %x - %x - %x\r\n\r\n",i,samplesU16[i*4],samplesU16[i*4+1],samplesU16[i*4+2],samplesU16[i*4+3]);
    for(int i=0;i<NLeqSlow;i++)
            pc.printf("[%d]: %04.1f ",i,LeqFastSamples[i]); 
    prevTime=currTime;
    tcount++;
} 

void s1Data(void const *args)
{
    //pc.printf("siDataId: %x\r\n",osThreadGetId ());
    while(1)
    {
        Thread::signal_wait(0x1); 
        led=!led;         
        auxSave++;
        if(LeqTime!=0) 
        {            
            if(LeqTime<=60) 
            {                                
                if(LeqSecondsCount>=LeqTime) 
                {                                    
                    LeqSecondsCount=0;                    
                    for(int i=0;i<NFilters;i++)
                        LeqAuxTOB[i] = sumaLeq(LeqSecondsTOB[i],LeqTime);                                    
                    saveData(sumaLeq(LeqSeconds,LeqTime),LeqAuxTOB);
                }
            } else {
                if(LeqSecondsCount>=60) {
                    LeqSecondsCount=0;
                    LeqMinutes[LeqMinutesCount]=sumaLeq(LeqSeconds,60);
                    
                    for(int i=0;i<NFilters;i++)
                        LeqMinutesTOB[i][LeqMinutesCount] = sumaLeq(LeqSecondsTOB[i],60);
                        
                    LeqMinutesCount++;
                }
                if(LeqMinutesCount>=(LeqTime/60)) {
                    LeqMinutesCount=0;
                    for(int i=0;i<NFilters;i++)
                        LeqAuxTOB[i] = sumaLeq(LeqMinutesTOB[i],LeqTime/60);
                    saveData(sumaLeq(LeqMinutes,(LeqTime/60)),LeqAuxTOB);
                }
            }
        }
    }
}
 
void startSampling()
{       
    if(gps)
    {
        while(GPSmodule.getFixQuality()<1)
        {
            if(GPSmodule.sampleGPS())
                printf("Waiting valid GPS signal: %d!\r\n",GPSmodule.getFixQuality());
        }
        GPSmodule.startGPS();
        ThGPS->signal_set(0x1); 
    }
    
    if(RecTime!=0)
    {
        char buffer[32];
        endTime =  rtc_read();
        strftime(buffer, 32, "%d/%m/%y %T", localtime(&endTime));    
        printf("Recording time init: %s\r\n ",buffer);     
        endTime=endTime+3600*RecTime;  
        strftime(buffer, 32, "%d/%m/%y %T", localtime(&endTime)); 
        printf("Recording time end: %s - %d\r\n", buffer, endTime); 
    }
    
    t.start();  //Debug for timing    
    LeqFastCount=0;   //Number of fast samples taken
    LeqFastTOBCount=0;     //Number of fast samples taken for third Octave Band
    LeqSecondsCount=0;
    LeqMinutesCount=0;
    ADCcount=0; 
    auxSave=0;       
    
    time_t seconds =  rtc_read();
    strftime(fileName, 32, "/sd/%d_%m_%y-%H_%M_%S.txt", localtime(&seconds));
    insertHeader();
    printf("\r\nFile: %s\r\n",fileName); //Debug file
    filePtr = fopen(fileName,"a");
    if(filePtr==NULL) 
    {
        printf("Error writing log\r\n");
        stopSampling();
    }
    
    if (initI2S()==HAL_I2S_STATE_ERROR ) //Problem with I2S port initialization
    {
        printf("Error init I2S\r\n");
        stopSampling();
        return;
    }
    else
    {
        __HAL_I2S_CLEAR_OVRFLAG(&MEMSmic);
        //HAL_I2S_Receive_DMA(&MEMSmic,&samplesU16[0],2);
        HAL_I2S_Receive_DMA(&MEMSmic,&samplesU16[0],N*2); 
    }
    Thread::wait(500);
    start = true;
    if(!RTC_1s())
    {
        printf("Error init RTC\r\n");
        stopSampling();
        return;  
    }
}

void HAL_I2S_MspDeInit(I2S_HandleTypeDef* i2sHandle)
{
  if(i2sHandle->Instance==SPI2)
  {
    __HAL_RCC_SPI2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  }
} 

void stopSampling()
{    
    sem125.wait(1000);
    HAL_RTC_DeactivateAlarm(&iRTC,RTC_ALARM_A);
    start=false;  
    tickADC125.detach();  //Stop 125ms thread ("fast")  
    GPSmodule.stopGPS();      
    fclose(filePtr); 
    HAL_I2S_MspDeInit(&MEMSmic);    
    printf("Stop\r\n");   //Debug stop
    led=0;
}

void selection(void const *args) 
{
  //pc.printf("SelectionId: %x\r\n",osThreadGetId ());
  while(1) {
    Thread::signal_wait(0x1);        
    if(!start)
        startSampling();
    else
        stopSampling();
    button.enable_irq();    
  }
}  
  
void usrBtnISR()
{
    ThStart->signal_set(0x1); 
    button.disable_irq();
}

bool loadValues()
{
    short c;    
    string data;
    char tmp;
    int i=0;
    filePtr = fopen("/sd/config.ini","r");
    if(filePtr!=NULL) 
    {
        do
        { // read all lines in file   
            i++;                 
            do
            {
                c = fgetc(filePtr); 
                data.push_back((char)c);
            }while(c != EOF && c!= ';'); 
            printf("Datos %d: %s \r\n",i,data.c_str());
            if (sscanf(data.c_str(), "fs=%d", &Fs) >= 1) {} 
            else if (sscanf(data.c_str(), "cal=%f", &cal) >= 1) {} 
            else if (sscanf(data.c_str(), "gps=%c", &tmp) >= 1) {gps=!(tmp=='F');} 
            else if (sscanf(data.c_str(), "LeqTime=%d", &LeqTime) >= 1) {} 
            else if (sscanf(data.c_str(), "RecTime=%d", &RecTime) >= 1) {} 
            else if (sscanf(data.c_str(), "PondFilter=%c", &PondFilter) >= 1) {} 
            else if (sscanf(data.c_str(), "octave=%c", &tmp) >= 1) {octave=!(tmp=='F');} 
            
            while((c = fgetc(filePtr))!= EOF && c!= '\n'){} //Find end of line or end of file            
            data.clear();
        }while(c!=EOF);
        /*gps=false;
        RecTime=0;
        LeqTime=1;
        octave=true;
        Fs=32000;*/
        fclose(filePtr); 
        return true;      
    }else
        return false;    
}

int initI2S()
{
    DMA_Init();
    //2.a Enable clocks to PORTB, PORTC and SPI2
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();
    __SPI2_CLK_ENABLE();

    /** 2.b I2S2 GPIO Configuration
    PC3     ------> I2S2_SD
    PB10 - D6     ------> I2S2_CK
    PB12     ------> I2S2_WS
    */

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
        //Set interrupt
    HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);

    // intialize I2S clocks

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;

    PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;    //Division factor for PLL VCO input clock
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 5;     //Specifies the division factor for I2S clock.
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;   //Specifies the multiplication factor for PLLI2S VCO output clock

    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    
    
    
    // set up I2S

    MEMSmic.Instance = SPI2;
    MEMSmic.Init.Mode = I2S_MODE_MASTER_RX;
    MEMSmic.Init.Standard = I2S_STANDARD_MSB;
    MEMSmic.Init.DataFormat = I2S_DATAFORMAT_24B;
    MEMSmic.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    MEMSmic.Init.AudioFreq = Fs; //I2S_AUDIOFREQ_32K;
    MEMSmic.Init.CPOL = I2S_CPOL_LOW;
    MEMSmic.Init.ClockSource = I2S_CLOCK_PLL;
    MEMSmic.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    HAL_I2S_Init(&MEMSmic);      
    
    /* Peripheral DMA init*/
  
    hdma_spi2_rx.Instance = DMA1_Stream3;
    hdma_spi2_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_rx.Init.Mode = DMA_NORMAL;
    hdma_spi2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
      printf("DMA error\r\n");      
    }

    __HAL_LINKDMA(&MEMSmic,hdmarx,hdma_spi2_rx);
    
    return HAL_I2S_GetState(&MEMSmic);
} 

time_t rtc_read(void)
{
    RTC_DateTypeDef dateStruct;
    RTC_TimeTypeDef timeStruct;
    struct tm timeinfo;
 
    iRTC.Instance = RTC;
 
    // Read actual date and time
    // Warning: the time must be read first!
    HAL_RTC_GetTime(&iRTC, &timeStruct, FORMAT_BIN);
    HAL_RTC_GetDate(&iRTC, &dateStruct, FORMAT_BIN);
 
    // Setup a tm structure based on the RTC
    timeinfo.tm_wday = dateStruct.WeekDay;
    timeinfo.tm_mon  = dateStruct.Month - 1;
    timeinfo.tm_mday = dateStruct.Date;
    timeinfo.tm_year = dateStruct.Year + 100;
    timeinfo.tm_hour = timeStruct.Hours;
    timeinfo.tm_min  = timeStruct.Minutes;
    timeinfo.tm_sec  = timeStruct.Seconds;
 
    // Convert to timestamp
    time_t t = mktime(&timeinfo);
 
    return t;
}

bool initRTC()
{    
    iRTC.Instance = RTC;
    iRTC.Init.HourFormat = RTC_HOURFORMAT_24;
    iRTC.Init.AsynchPrediv = 127;
    iRTC.Init.SynchPrediv = 255;
    iRTC.Init.OutPut = RTC_OUTPUT_DISABLE;
    iRTC.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    iRTC.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);  //HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
    if (HAL_RTC_Init(&iRTC) != HAL_OK)
        return false;
    else 
        return true;
}

void HAL_RTC_AlarmAEventCallback (RTC_HandleTypeDef *hrtc)
{
    tickADC125.attach(ticker125ISR,0.125); 
    //printf("****Alarm\r\n");    
}

bool RTC_1s(void)
{
  RTC_AlarmTypeDef salarmstructure;

  salarmstructure.Alarm = RTC_ALARM_A;
  salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_TUESDAY;
  salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  salarmstructure.AlarmMask = RTC_ALARMMASK_ALL;   //Which data to not take into account (ALL - 1s alarm)
  salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  salarmstructure.AlarmTime.Hours = 0x12;
  salarmstructure.AlarmTime.Minutes = 0x00;
  salarmstructure.AlarmTime.Seconds = 0x00;
  salarmstructure.AlarmTime.SubSeconds = 0x00;
  
    if(HAL_RTC_SetAlarm_IT(&iRTC,&salarmstructure,FORMAT_BCD) != HAL_OK)
        return false;
    else
        return true;
}

int init()
{            
    //Status
    // 1 - FFT
    // 2 - Wrong Leq time
    // 3 - Error loading file
    // 4 - No SD in
    // 5 - MEMS port 
    // 6 - Internal RTC
    
    SDin.mode(PullUp);  //SD detection pin - pull up    
    status = ARM_MATH_SUCCESS;  //Requeriment for FFT calculation
    status = arm_rfft_fast_init_f32(&S,N);  //Requeriment for FFT calculation
    if(status!=0)   //FFT init not performed properly
        return 1;
        
    if (initI2S()==HAL_I2S_STATE_ERROR ) //Problem with I2S port initialization
        return 5;
    //else
        //HAL_I2S_Receive(&MEMSmic,&samplesU16[0],2,0x0);    
    if (!initRTC())
        return 6;   
    if(SDin)    //Detect if SD is in
    {
        if(!button)
        {
            mainThreadID = osThreadGetId();
            return -1;
        } else if(!loadValues())   //Load .ini values
            return 3; //pc.printf("Error opening file");
    }
    else
        return 4; //pc.printf("SD not detected");        
    if(LeqTime>3600)
        return 2;            
        
    button.fall(&usrBtnISR);    //Attach function for button interrupt when pressed 0->1   
    cte=2.0f/((1.0f/Fs)*N);     //Compute the constant value to calculate Pxx
    return 0;
}

void Calibrate(void const *args)
{    
    //tickADC125.attach(ticker125ISR,0.125);  //Start 125ms thread ("fast")    
    if(!RTC_1s())
    {
        printf("Error init RTC\r\n");
        stopSampling();
        return;  
    }
    Thread::wait(1);
    while(true)
    {        
        sem125.wait();
                
        float Pxx = 0.0;    //Calculate power
        double Pxf = 0.0;   //Power in dB    
        float auxF=0;

        for(int i=0; i<N; i++) 
        {            
            //pc.printf("Received0[%d]: %x - %x - %x - %x\r\n",i,samplesU16[i*4],samplesU16[i*4+1],samplesU16[i*4+2],samplesU16[i*4+3]);            
            
            auxU162[0] = 0;
            auxU162[1] = 0;
            
            switch(mic)
            {
                case 0:
                    for(int j=0; j<4; j++) 
                    {
                        if((samplesU16[i*4+j] & 0x1FFE)!=0) 
                        {
                            auxU162[0] = (samplesU16[i*4+j]& 0x7FFF);    
                        }else if(samplesU16[i*4+j]!=0)
                        {
                            auxU162[1] = samplesU16[i*4+j];
                        } 
                    } 
                                           
                    auxF = (float)(( (auxU162[0]<<2) | (auxU162[1]>>14 & 0x0003) ) & 0x0003FFFF);
                    if(auxF>65536)
                        auxSamples[i] = (auxF - 65536);
                    else
                        auxSamples[i] = (65536 + auxF);
                break;
                
                case 1:
                    for(int j=0; j<4; j++) 
                    {
                        if((samplesU16[i*4+j] & 0x007F)!=0) 
                            auxU162[0] = samplesU16[i*4+j] & 0x7FFF;    
                        else if(samplesU16[i*4+j]!=0)
                            auxU162[1] = samplesU16[i*4+j] & 0xFF80;
                    }                                            
                    auxF = (((auxU162[0]<<9 & 0x00FFFE00) | (auxU162[1]>>7 & 0x000001FF)) & 0x00FFFFFF);
                    if(auxF>8388608)
                        auxSamples[i] = (auxF - 8388608);
                    else
                        auxSamples[i] = (8388608 + auxF);
                break;
            }
        }
        
        arm_mean_f32(auxSamples, N, &auxF);         
        for(int i=0;i<N;i++)
            auxSamples[i]=auxSamples[i]-auxF;
            
        arm_biquad_cascade_df2T_init_f32(&iirS, NUM_SECTIONS, AFilterCoeff, iir_state);    
        if(PondFilter=='A')
            arm_biquad_cascade_df2T_f32Mod(&iirS, auxSamples, outFFT, N, AFilterGains); // A Filter                              
        arm_rms_f32(outFFT,N,&Pxx);
        Pxf = 20*log10(Pxx);        
        calData[calCount]=Pxf;            
        pc.printf("Cal[%d] = %f \r\n",calCount,calData[calCount]);
        calCount++;        
        
        if(calCount>=CalNSamples)
        {
            float average=0;
            for(int i=1; i<CalNSamples; i++)
                average+=calData[i];
            cal=94.0f-(average/(CalNSamples-1));  //80*125ms = 10 seconds
            pc.printf("Final Cal = %f \r\n",cal);
            setCal(cal);          
            osSignalSet(mainThreadID, 0x1);
        }
    }
}

void setCal(float value)
{
    short c; 
    short line=2;
    char buffer[20];
    sprintf (buffer, "%09.5f", value);  //Set cal value with format 000.00000
    filePtr = fopen("/sd/config.ini","r+");
    if(filePtr) 
    {
        for(int i=0;i<line;i++)
        { // read all lines in file                    
            if(i<(line-1))  //Jump lines until requested 
                do{c = fgetc(filePtr); }while(c != EOF && c!= '\n'); 
            else 
            {
                while((c = fgetc(filePtr))!= EOF && c!= '=' && c != '%'){}  //Find value
                for(short i=0;i<9;i++)
                    fputc(buffer[i],filePtr); 
            }
        }
        fclose(filePtr);       
    }else printf("Write failed");
}

void DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PLL_PLLM_CONFIG(16);

  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

int main() 
{
    HAL_Init();    
    //SystemClock_Config();
    pc.baud(230400);
    
    pc.printf("\r\n\r\nInit...\r\n");

    switch(init())
    {
        case 0:
 
        break;     
        case 1:
            pc.printf("Error in fft parameters initialization");
            Thread::wait(1);
            for(int i=0;i<=1;i++)
            {
                led=!led;
                Thread::wait(500); 
            }
            led=0;
            Thread::wait(2);
            errorTick.attach(errorISR,0.1);
            while(1);
            
        case 2:
            pc.printf("Measurement time - LeqTime>3600");
            Thread::wait(1);
            for(int i=0;i<=2;i++)
            {
                led=!led;
                Thread::wait(500);
            }
            led=0;
            Thread::wait(2);
            errorTick.attach(errorISR,0.1);
            while(1);
            
        case 3:
            pc.printf("Error opening configuration file");
            Thread::wait(1);
            for(int i=0;i<=3;i++)
            {
                led=!led;
                Thread::wait(500);
            }
            led=0;
            Thread::wait(2);
            errorTick.attach(errorISR,0.1);
            while(1);
            
        case 4:
            pc.printf("SD not detected");
            Thread::wait(1);
            for(int i=0;i<=4;i++)
            {
                led=!led;
                Thread::wait(500);
            }
            led=0;
            Thread::wait(2);
            errorTick.attach(errorISR,0.1);
            while(1);
            
        case 5:
            pc.printf("Microphone init failed");
            Thread::wait(1);
            for(int i=0;i<=5;i++)
            {
                led=!led;
                Thread::wait(500);
            }
            led=0;
            Thread::wait(2);
            errorTick.attach(errorISR,0.1);
            while(1);
            
        case 6:
            pc.printf("RTC init failed");
            Thread::wait(1);
            for(int i=0;i<=6;i++)
            {
                led=!led;
                Thread::wait(500);
            }
            led=0;
            Thread::wait(2);
            errorTick.attach(errorISR,0.1);
            while(1);
        
        case -1:
            printf("\r\nCalibration started...\r\n");    //Debug - calibrating   
            __HAL_I2S_CLEAR_OVRFLAG(&MEMSmic);
            HAL_I2S_Receive_DMA(&MEMSmic,&samplesU16[0],N*2);             
            ThCalibrate =  new Thread(Calibrate,(void *)"ThCal",osPriorityHigh, 2048, NULL);    //Thread to compute 1s data                         
            osSignalWait(0x1, osWaitForever); //Wait for calibration process to finish
            ThCalibrate->terminate();  //Calibration done 
            stopSampling(); //Calibration done  
            init();                                 
        break;
    }
    
    led=1;
    Thread::wait(1000);
    led=0;
    
    ThStart = new Thread(selection,(void *)"ThSt",osPriorityAboveNormal, 2048, NULL);    //Thread to control measurement init - osPriorityAboveNormal
    ThFastComp = new Thread(computations,(void *)"ThCom",osPriorityRealtime, 2048, NULL);    //Thread to compute PSD's - osPriorityRealtime
    Th1sData =  new Thread(s1Data,(void *)"Th1s",osPriorityHigh, 8192, NULL);    //Thread to compute 1s data - osPriorityHigh
    ThAuxSave =  new Thread(update,(void *)"ThAux",osPriorityNormal, 2048, NULL);    //Thread to save data - osPriorityNormal
    ThGPS =  new Thread(getGPSData,(void *)"ThGPS",osPriorityLow, 2048, NULL);    //Thread to save data - osPriorityLow
    
    pc.printf("Iniciado\r\n");
    led=0;            
    
    Thread::wait(osWaitForever);

}

float sumaLeq(double *InputData,int inSize)
{
    float retVal=0;
    for(int i=0;i<inSize;i++)
    {
        //if(inSize!=8)printf("Val[%d]= %f\r\n",i,InputData[i]);
        retVal=retVal+(float)pow(10.0,((double)InputData[i]/10.0));
    }        
    return 10.0*log10((double)retVal/inSize);
}

void mult_f32(
  float32_t * pSrcA,
  const float * pSrcB,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counters */
#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */
  float32_t inA1, inA2, inA3, inA4;              /* temporary input variables */
  float32_t inB1, inB2, inB3, inB4;              /* temporary input variables */
  float32_t out1, out2, out3, out4;              /* temporary output variables */

  /* loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.        
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = A * B */
    /* Multiply the inputs and store the results in output buffer */
    /* read sample from sourceA */
    inA1 = *pSrcA;
    /* read sample from sourceB */
    inB1 = *pSrcB;
    /* read sample from sourceA */
    inA2 = *(pSrcA + 1);
    /* read sample from sourceB */
    inB2 = *(pSrcB + 1);

    /* out = sourceA * sourceB */
    out1 = inA1 * inB1;

    /* read sample from sourceA */
    inA3 = *(pSrcA + 2);
    /* read sample from sourceB */
    inB3 = *(pSrcB + 2);

    /* out = sourceA * sourceB */
    out2 = inA2 * inB2;

    /* read sample from sourceA */
    inA4 = *(pSrcA + 3);

    /* store result to destination buffer */
    *pDst = out1;

    /* read sample from sourceB */
    inB4 = *(pSrcB + 3);

    /* out = sourceA * sourceB */
    out3 = inA3 * inB3;

    /* store result to destination buffer */
    *(pDst + 1) = out2;

    /* out = sourceA * sourceB */
    out4 = inA4 * inB4;
    /* store result to destination buffer */
    *(pDst + 2) = out3;
    /* store result to destination buffer */
    *(pDst + 3) = out4;


    /* update pointers to process next samples */
    pSrcA += 4u;
    pSrcB += 4u;
    pDst += 4u;

    /* Decrement the blockSize loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.        
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

#endif /* #ifndef ARM_MATH_CM0_FAMILY */

  while(blkCnt > 0u)
  {
    /* C = A * B */
    /* Multiply the inputs and store the results in output buffer */
    *pDst++ = (*pSrcA++) * (*pSrcB++);

    /* Decrement the blockSize loop counter */
    blkCnt--;
  }
}

LOW_OPTIMIZATION_ENTER
void arm_biquad_cascade_df2T_f32Mod(
const arm_biquad_cascade_df2T_instance_f32 * S,
float32_t * pSrc,
float32_t * pDst,
uint32_t blockSize, const float * gainVec)
{

   float32_t *pIn = pSrc;                         /*  source pointer            */
   float32_t *pOut = pDst;                        /*  destination pointer       */
   float32_t *pState = S->pState;                 /*  State pointer             */
   float32_t *pCoeffs = S->pCoeffs;               /*  coefficient pointer       */
   float32_t acc1;                                /*  accumulator               */
   float32_t b0, b1, b2, a1, a2;                  /*  Filter coefficients       */
   float32_t Xn1;                                 /*  temporary input           */
   float32_t d1, d2;                              /*  state variables           */
   uint32_t sample, stage = S->numStages;         /*  loop counters             */
    unsigned short gainFlg = 0;
#if defined(ARM_MATH_CM7)
    
   float32_t Xn2, Xn3, Xn4, Xn5, Xn6, Xn7, Xn8;   /*  Input State variables     */
   float32_t Xn9, Xn10, Xn11, Xn12, Xn13, Xn14, Xn15, Xn16;
   float32_t acc2, acc3, acc4, acc5, acc6, acc7;  /*  Simulates the accumulator */
   float32_t acc8, acc9, acc10, acc11, acc12, acc13, acc14, acc15, acc16;

   do
   {
      /* Reading the coefficients */ 
      b0 = pCoeffs[0]; 
      b1 = pCoeffs[1]; 
      b2 = pCoeffs[2]; 
      a1 = pCoeffs[3]; 
      /* Apply loop unrolling and compute 16 output values simultaneously. */ 
      sample = blockSize >> 4u; 
      a2 = pCoeffs[4]; 

      /*Reading the state values */ 
      d1 = pState[0]; 
      d2 = pState[1]; 

      pCoeffs += 5u;

      
      /* First part of the processing with loop unrolling.  Compute 16 outputs at a time.       
       ** a second loop below computes the remaining 1 to 15 samples. */
      while(sample > 0u) {

         /* y[n] = b0 * x[n] + d1 */
         /* d1 = b1 * x[n] + a1 * y[n] + d2 */
         /* d2 = b2 * x[n] + a2 * y[n] */

         /* Read the first 2 inputs. 2 cycles */
         Xn1  = pIn[0 ];
         Xn2  = pIn[1 ];

         /* Sample 1. 5 cycles */
         Xn3  = pIn[2 ];
         acc1 = b0 * Xn1 + d1;
         
         Xn4  = pIn[3 ];
         d1 = b1 * Xn1 + d2;
         
         Xn5  = pIn[4 ];
         d2 = b2 * Xn1;
         
         Xn6  = pIn[5 ];
         d1 += a1 * acc1;
         
         Xn7  = pIn[6 ];
         d2 += a2 * acc1;

         /* Sample 2. 5 cycles */
         Xn8  = pIn[7 ];
         acc2 = b0 * Xn2 + d1;
         
         Xn9  = pIn[8 ];
         d1 = b1 * Xn2 + d2;
         
         Xn10 = pIn[9 ];
         d2 = b2 * Xn2;
         
         Xn11 = pIn[10];
         d1 += a1 * acc2;
         
         Xn12 = pIn[11];
         d2 += a2 * acc2;

         /* Sample 3. 5 cycles */
         Xn13 = pIn[12];
         acc3 = b0 * Xn3 + d1;
         
         Xn14 = pIn[13];
         d1 = b1 * Xn3 + d2;
         
         Xn15 = pIn[14];
         d2 = b2 * Xn3;
         
         Xn16 = pIn[15];
         d1 += a1 * acc3;
         
         pIn += 16;
         d2 += a2 * acc3;

         /* Sample 4. 5 cycles */
         acc4 = b0 * Xn4 + d1;
         d1 = b1 * Xn4 + d2;
         d2 = b2 * Xn4;
         d1 += a1 * acc4;
         d2 += a2 * acc4;

         /* Sample 5. 5 cycles */
         acc5 = b0 * Xn5 + d1;
         d1 = b1 * Xn5 + d2;
         d2 = b2 * Xn5;
         d1 += a1 * acc5;
         d2 += a2 * acc5;

         /* Sample 6. 5 cycles */
         acc6 = b0 * Xn6 + d1;
         d1 = b1 * Xn6 + d2;
         d2 = b2 * Xn6;
         d1 += a1 * acc6;
         d2 += a2 * acc6;

         /* Sample 7. 5 cycles */
         acc7 = b0 * Xn7 + d1;
         d1 = b1 * Xn7 + d2;
         d2 = b2 * Xn7;
         d1 += a1 * acc7;
         d2 += a2 * acc7;

         /* Sample 8. 5 cycles */
         acc8 = b0 * Xn8 + d1;
         d1 = b1 * Xn8 + d2;
         d2 = b2 * Xn8;
         d1 += a1 * acc8;
         d2 += a2 * acc8;

         /* Sample 9. 5 cycles */
         acc9 = b0 * Xn9 + d1;
         d1 = b1 * Xn9 + d2;
         d2 = b2 * Xn9;
         d1 += a1 * acc9;
         d2 += a2 * acc9;

         /* Sample 10. 5 cycles */
         acc10 = b0 * Xn10 + d1;
         d1 = b1 * Xn10 + d2;
         d2 = b2 * Xn10;
         d1 += a1 * acc10;
         d2 += a2 * acc10;

         /* Sample 11. 5 cycles */
         acc11 = b0 * Xn11 + d1;
         d1 = b1 * Xn11 + d2;
         d2 = b2 * Xn11;
         d1 += a1 * acc11;
         d2 += a2 * acc11;

         /* Sample 12. 5 cycles */
         acc12 = b0 * Xn12 + d1;
         d1 = b1 * Xn12 + d2;
         d2 = b2 * Xn12;
         d1 += a1 * acc12;
         d2 += a2 * acc12;

         /* Sample 13. 5 cycles */
         acc13 = b0 * Xn13 + d1;         
         d1 = b1 * Xn13 + d2;         
         d2 = b2 * Xn13;
         
         pOut[0 ] = acc1 ;
         d1 += a1 * acc13;
         
         pOut[1 ] = acc2 ;  
         d2 += a2 * acc13;

         /* Sample 14. 5 cycles */
         pOut[2 ] = acc3 ;  
         acc14 = b0 * Xn14 + d1;
             
         pOut[3 ] = acc4 ;
         d1 = b1 * Xn14 + d2;
          
         pOut[4 ] = acc5 ; 
         d2 = b2 * Xn14;
         
         pOut[5 ] = acc6 ;    
         d1 += a1 * acc14;
         
         pOut[6 ] = acc7 ;  
         d2 += a2 * acc14;

         /* Sample 15. 5 cycles */
         pOut[7 ] = acc8 ;
         pOut[8 ] = acc9 ;  
         acc15 = b0 * Xn15 + d1;
              
         pOut[9 ] = acc10;  
         d1 = b1 * Xn15 + d2;
         
         pOut[10] = acc11;  
         d2 = b2 * Xn15;
         
         pOut[11] = acc12;
         d1 += a1 * acc15;
         
         pOut[12] = acc13;
         d2 += a2 * acc15;

         /* Sample 16. 5 cycles */
         pOut[13] = acc14;  
         acc16 = b0 * Xn16 + d1;
         
         pOut[14] = acc15;  
         d1 = b1 * Xn16 + d2;
         
         pOut[15] = acc16;
         d2 = b2 * Xn16;
         
         sample--;   
         d1 += a1 * acc16;
         
         pOut += 16;
         d2 += a2 * acc16;
      }

      sample = blockSize & 0xFu;
      while(sample > 0u) {
         Xn1 = *pIn;         
         acc1 = b0 * Xn1 + d1;
         
         pIn++;
         d1 = b1 * Xn1 + d2;
         
         *pOut = acc1; 
         d2 = b2 * Xn1;
         
         pOut++;
         d1 += a1 * acc1;
         
         sample--;  
         d2 += a2 * acc1; 
      }

      /* Store the updated state variables back into the state array */ 
      pState[0] = d1; 
      /* The current stage input is given as the output to the next stage */ 
      pIn = pDst; 
      
      pState[1] = d2; 
      /* decrement the loop counter */ 
      stage--; 

      pState += 2u;

      /*Reset the output working pointer */ 
      pOut = pDst; 

   } while(stage > 0u);
    
#elif defined(ARM_MATH_CM0_FAMILY)

   /* Run the below code for Cortex-M0 */

   do
   {
      /* Reading the coefficients */
      b0 = *pCoeffs++;
      b1 = *pCoeffs++;
      b2 = *pCoeffs++;
      a1 = *pCoeffs++;
      a2 = *pCoeffs++;

      /*Reading the state values */
      d1 = pState[0];
      d2 = pState[1];


      sample = blockSize;

      while(sample > 0u)
      {
         /* Read the input */
         Xn1 = *pIn++;

         /* y[n] = b0 * x[n] + d1 */
         acc1 = (b0 * Xn1) + d1;

         /* Store the result in the accumulator in the destination buffer. */
         *pOut++ = acc1;

         /* Every time after the output is computed state should be updated. */
         /* d1 = b1 * x[n] + a1 * y[n] + d2 */
         d1 = ((b1 * Xn1) + (a1 * acc1)) + d2;

         /* d2 = b2 * x[n] + a2 * y[n] */
         d2 = (b2 * Xn1) + (a2 * acc1);

         /* decrement the loop counter */
         sample--;
      }

      /* Store the updated state variables back into the state array */
      *pState++ = d1;
      *pState++ = d2;

      /* The current stage input is given as the output to the next stage */
      pIn = pDst;

      /*Reset the output working pointer */
      pOut = pDst;

      /* decrement the loop counter */
      stage--;

   } while(stage > 0u);
     
#else

   float32_t Xn2, Xn3, Xn4;                       /*  Input State variables     */
   float32_t acc2, acc3, acc4;                        /*  accumulator               */


   float32_t p0, p1, p2, p3, p4, A1;

   /* Run the below code for Cortex-M4 and Cortex-M3 */
   do
   {
      /* Reading the coefficients */     
      b0 = *pCoeffs++;
      b1 = *pCoeffs++;
      b2 = *pCoeffs++;
      a1 = *pCoeffs++;
      a2 = *pCoeffs++;
      

      /*Reading the state values */
      d1 = pState[0];
      d2 = pState[1];

      /* Apply loop unrolling and compute 4 output values simultaneously. */
      sample = blockSize >> 2u;

      /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.       
   ** a second loop below computes the remaining 1 to 3 samples. */
      while(sample > 0u) {

         /* y[n] = b0 * x[n] + d1 */
         /* d1 = b1 * x[n] + a1 * y[n] + d2 */
         /* d2 = b2 * x[n] + a2 * y[n] */

         /* Read the four inputs */
         Xn1 = pIn[0];
         Xn2 = pIn[1];
         Xn3 = pIn[2];
         Xn4 = pIn[3];
         pIn += 4;     

         p0 = b0 * Xn1; 
         p1 = b1 * Xn1;
         acc1 = p0 + d1;
         p0 = b0 * Xn2; 
         p3 = a1 * acc1;
         p2 = b2 * Xn1;
         A1 = p1 + p3;
         p4 = a2 * acc1;
         d1 = A1 + d2;
         d2 = p2 + p4;

         p1 = b1 * Xn2;
         acc2 = p0 + d1;
         p0 = b0 * Xn3;  
         p3 = a1 * acc2; 
         p2 = b2 * Xn2;                                 
         A1 = p1 + p3;
         p4 = a2 * acc2;
         d1 = A1 + d2;
         d2 = p2 + p4;

         p1 = b1 * Xn3;
         acc3 = p0 + d1;
         p0 = b0 * Xn4; 
         p3 = a1 * acc3;
         p2 = b2 * Xn3;
         A1 = p1 + p3;
         p4 = a2 * acc3;
         d1 = A1 + d2;
         d2 = p2 + p4;

         acc4 = p0 + d1;
         p1 = b1 * Xn4;
         p3 = a1 * acc4;
         p2 = b2 * Xn4;
         A1 = p1 + p3;
         p4 = a2 * acc4;
         d1 = A1 + d2;
         d2 = p2 + p4;

         pOut[0] = acc1*gainVec[gainFlg];    
         pOut[1] = acc2*gainVec[gainFlg];    
         pOut[2] = acc3*gainVec[gainFlg];    
         pOut[3] = acc4*gainVec[gainFlg];
         pOut += 4;
                        
         sample--;         
      }

      sample = blockSize & 0x3u;
      while(sample > 0u) {
         Xn1 = *pIn++;

         p0 = b0 * Xn1; 
         p1 = b1 * Xn1;
         acc1 = p0 + d1;
         p3 = a1 * acc1;
         p2 = b2 * Xn1;
         A1 = p1 + p3;
         p4 = a2 * acc1;
         d1 = A1 + d2;
         d2 = p2 + p4;
    
         *pOut++ = acc1*gainVec[gainFlg];
         
         sample--;         
      }

      /* Store the updated state variables back into the state array */
      *pState++ = d1;
      *pState++ = d2;

      /* The current stage input is given as the output to the next stage */
      pIn = pDst;

      /*Reset the output working pointer */
      pOut = pDst;

      /* decrement the loop counter */
      stage--;
      gainFlg++; 
   } while(stage > 0u);

#endif 

}
LOW_OPTIMIZATION_EXIT