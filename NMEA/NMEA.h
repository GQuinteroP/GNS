#ifndef NMEA_H
#define NMEA_H
#include "mbed.h"
#include "rtos.h"

#define buffSize 1500


class GPS {
public:   
    GPS(PinName tx, PinName rx,rtos::Semaphore *semGPS);
    ~GPS();
    void startGPS();  
    void stopGPS(); 
    float getTime();
    float getLon();
    float getLat();   
    int getFixQuality();
    int getSatellites();
    char pollData();
    bool readableData();
    bool sampleGPS(); 
    void sampleGPSISR(); 
    
    float nmeaToDec(float deg_coord, char nsew);
private:    
    rtos::Semaphore *sGPS;
    void processNMEA();
    Serial gps;    
    Timeout resTimeout;
    char GPSStr[1024];
    int GPSCount;
    bool actBuff;
    unsigned int charCount;
    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;    
    float utc_time;
    char ns, ew;
    int lock;
    int satellites;
    float hdop;
    float msl_altitude;
    char msl_units;
    
    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;
    
    // GLL
    char gll_status;
    
    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
};
#endif