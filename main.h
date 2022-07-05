#define N 4096 // 33kHz - 125 ms, if we wanted to improve FS, we need to search for another fft which accept more than 4096 data
#define NLeqSlow 8  //Number of leq needed to get 1s sampling according to FS/N
#define initError 0.1   //Time to blink led to represent an error in initialization

#define CalNSamples 80  //10 seconds
#define debug true


//Debugging

#define f 1000  //Debugging - generate 1khz sin
const float pi=3.1415926535897932384;   //Pi value for debuggin purposes
Timer t;
int prevTime=0;
int currTime=0;
int tcount=1;
float rel=0;
int tmpCount=0;

Serial pc(SERIAL_TX, SERIAL_RX);


//Threads functions
void computations(void const *args);
void selection(void const *args);
void s1Data(void const *args);
void update(void const *args);
void getGPSData(void const *args);
void Calibrate(void const *args);

//Threads
Thread *ThFastComp;    //Thread to compute PSD's
Thread *ThStart;    //Thread to control measurement init
Thread *Th1sData; //Thread to store data in SD
Thread *ThCalibrate; //Thread to store data in SD
Thread *ThAuxSave; //Thread to store data in SD
Thread *ThGPS;  //Thread to get GPS data
osThreadId mainThreadID;

Semaphore sem125(0);   //Controls sampling process
Semaphore semSave(0); // Called every second
Semaphore semGPS(0);    //Control gps sampling

//Init functions
int initI2S();
bool initRTC();
void DMA_Init(void);
time_t rtc_read(void);
bool RTC_1s(void);

//Other required functions

void mult_f32( float32_t * pSrcA,const float * pSrcB,float32_t * pDst,uint32_t blockSize);  //Function overloaded to accept const float (A/C filter)
void arm_biquad_cascade_df2T_f32Mod(const arm_biquad_cascade_df2T_instance_f32 * S, float32_t * pSrc,float32_t * pDst,uint32_t blockSize, const float * gainVec);
void stopSampling();
void setCal(float value);
float sumaLeq(double *InputData,int inSize);

int Fs=33333;   //sampling frequency
float cal=31.01580; //calibration value
bool gps=true;   //Use gps coordinates
int LeqTime=1;  //Leq time period - '0' value indicates fast Leq
int RecTime=1;    //Recording time - '0' value indicates push button control recording
char PondFilter='A';  //'A' or 'C' ponderation filter
bool octave=true;  //'True' or 'false' calculate octave bands
char fileName[32];
time_t endTime;     //End time of recording
double calData[CalNSamples];    //10 values to get calibation value from average
short calCount=0;   //Calibration values counter
bool calibrate=false;   //Indicates calibration requested

uint16_t samplesU16[N*4];
uint16_t auxU162[4] = {0xF0F0, 0x5555, 0x0F0F, 0xFFFF};
float auxSamples[N];   //ADC samples auxiliar array to continue sampling in samples[N]
float cte=0;       //Constant value to compute Pxx (cte=2.0f/((1.0f/Fs)*N); )
float outFFT[N];  //Out var for FFT calculations
double LeqFastTOB[NFilters][NLeqSlow];   //Third octave band levels for fast sampling
double LeqFastSamples[NLeqSlow];   //Save 8 Leq samples to get slow Leq
bool LeqFastFull=false; //Flag, ready to compute slow Leq
short LeqFastCount=0;   //Number of fast samples taken
short LeqFastTOBCount=0;     //Number of fast samples taken for third Octave Band
double LeqAuxTOB[NFilters];    //Leq value computed for third octave bands to be saved
double LeqSeconds[60];  //60 seconds values
double LeqSecondsTOB[NFilters][60];  //60 seconds values
short LeqSecondsCount=0;  //Count of number of seconds got
double LeqMinutes[60];  // 60 minutes values
double LeqMinutesTOB[NFilters][60];  // 60 minutes values
short LeqMinutesCount=0;  // Count of number of minutes
int auxSave=0;  //Seconds cpo
int limit=900;  //Save samples every 15 min
short  ADCcount=0;     //ADC samples number
bool start=false;   //Boolean value to start Measuring
char GPSStr[1024];
int GPSCount;

arm_rfft_fast_instance_f32 S;   //Requirement for fft
arm_status status;  //Requirement for fft
Ticker tickADC125;  //Sampling sync tick
Ticker errorTick;   //Error ticker
InterruptIn button(USER_BUTTON);    //Push button to start measuring
DigitalOut led(D9);   //Led to show errors
SDFileSystem sd(D11, D12, D13, D10, "sd");  // MicroSD - DI, DO, SCK, CS
DigitalIn SDin(D7);       //Pin for sd card detection
FILE *filePtr;              // File pointer declare
GPS GPSmodule(D8, D2, &semGPS); //GPS module conection
//AnalogIn ADCvalue(A0);  //Get samples in A0 input
I2S_HandleTypeDef MEMSmic;    // I2S port to get data from MEMS microphone
RTC_HandleTypeDef iRTC;
DMA_HandleTypeDef hdma_spi2_rx;
arm_biquad_cascade_df2T_instance_f32 iirS;
float32_t iir_state[4*NUM_SECTIONS];

