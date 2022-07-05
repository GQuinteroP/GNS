#include "NMEA.h"


GPS::GPS(PinName tx, PinName rx, rtos::Semaphore *semGPS): gps(tx,rx)
{    
    charCount=0;
    actBuff=false;
    sGPS=semGPS;
}

GPS::~GPS(){}

void GPS::startGPS()
{
    this->gps.attach(this,&GPS::sampleGPSISR);
}

void GPS::stopGPS()
{
    this->gps.attach(NULL, Serial::RxIrq);
}

void GPS::sampleGPSISR()
{
    if(this->gps.readable()) 
        sGPS->release();        
}

bool GPS::sampleGPS()
{
    GPSStr[GPSCount]=gps.getc();
    GPSCount++;
    if(GPSStr[GPSCount-1]=='\n' || GPSCount>1023) {
        processNMEA();
        GPSCount=0;
        return true;
    }
    return false;
}

void GPS::processNMEA()
{
    if (sscanf(GPSStr, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &utc_time, &nmea_latitude, &ns, &nmea_longitude, &ew, &lock, &satellites, &hdop, &msl_altitude, &msl_units) >= 1)
        {}
    // Check if it is a GPRMC msg
    else if (sscanf(GPSStr, "$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f,%d", &utc_time,&rmc_status,&nmea_latitude, &ns, &nmea_longitude, &ew, &speed_k, &course_d, &date) >= 1)
        {}
    // GLL - Geographic Position-Lat/Lon
    else if (sscanf(GPSStr, "$GPGLL,%f,%c,%f,%c,%f,%c", &nmea_latitude, &ns, &nmea_longitude, &ew, &utc_time, &gll_status) >= 1)
        {}
}

float GPS::getTime()
{
    return utc_time;
}

float GPS::getLon()
{
    return nmeaToDec(nmea_longitude,ew);
}    

float GPS::getLat()
{
    return nmeaToDec(nmea_latitude,ns);
}   


int GPS::getFixQuality()
{
    return lock;
   //return 1; 
}  

int GPS::getSatellites()
{
    return satellites;
} 
 
bool GPS::readableData()
{
    if(this->gps.readable()) 
        return true;
    else
        return false;
}

char GPS::pollData()
{
    return gps.getc();
}

float GPS::nmeaToDec(float deg_coord, char nsew)
{
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}