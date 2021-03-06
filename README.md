# GNS (Georeferenced Noise Sensor)

A low-cost Georeferenced Noise Sensor (GNS) based on an ARM32 microcontroller and a digital MEMS microphone. The device is designed to perform georeferenced noise measurements of Leq, LAeq or Third Octave Bands for a frequency rango between 63 Hz - 10 kHz. The integration time, ponderation filter, sampling time, can be configured in config.ini file within the MicroSD.
 
The main components of the Hardware of the GNS are:
 
- STM32F411RE: Microcontroller with evaluation board.
- SPH0645LM4H-B: Digital MEMS microphone with evaluation board.
- RXM-GPS-RM-B: GPS Module.
- Micro SD 16 GB Micro SD memory.

<img src="/figures/GNS_MAIN.jpg" width="155" height="126"> <img src="/figures/GNS_cables.jpg" width="175" height="126"> <img src="/figures/GNS_mics.jpg" width="185" height="126">

## Documentation
The coding part of the present project was developed in Mbed Online Compiler (https://os.mbed.com/handbook/mbed-Compiler), platform which is already deprecated. The author has already succesfuly compiled the code in  Keil Studio Cloud (https://studio.keil.arm.com), but the output binary hasn't been tested in the hardware.

The aim of uploading this project is to port it to a new platform. Thus, to increase the capabilities, the target microcontroller would be the ESP32 from ESPRESSIF SYSTEMS.

### Software
The software is based on Mbed-rtos, it uses CMSIS library for signal filtering and mathematical operations, a very basic NMEA library to obtain GPS position and a SDLibrary (Neil Thiessen http://www.apache.org/licenses/LICENSE-2.0). 

There are three main tasks managed by the Mbed-rtos:
- Data sampling: Highest prioroty task which performs the data acquisition from the MEMS microphone continously, handled by DMA.
- Signal processing: Performs the filtering operations and indicators calculation (each 125 ms).
- Georeferencing and storage: Obtains the position of the GPS, performs the indicator integration ($\geq$ 1 s) and stores the data into the MicroSD.

For more information please refer to the paper stated in Citation section or e-mail me directly to guillermo.quintero.p@gmail.com.

### Hardware
The documentation to build-up a new measurement station can be found at [Hardware](/Hardware/Readme.md).

## Citation

Please reference the project as follows:

Quintero, G., Balastegui, A., & Romeu, J. (2019). A low-cost noise measurement device for noise mapping based on mobile sampling. Measurement: Journal of the International Measurement Confederation, 148, 106894. https://doi.org/10.1016/j.measurement.2019.106894
