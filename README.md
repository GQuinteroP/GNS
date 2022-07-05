# GNS
 A low-cost Georeferenced Noise Sensor based on an ARM32 microcontroller and a digital MEMS microphone. The main components are:
 
- STM32F411RE: Microcontroller with evaluation board.
- SPH0645LM4H-B: Digital MEMS microphone with evaluation board.
- RXM-GPS-RM-B: GPS Module.
- Passive antenna GPS antenna with micro coaxial connector.
- MIC5504-3.3YM5-TR: LDO voltage regulator.
- Micro SD 16 GB Micro SD memory.
- Connectors and case Coin cell, AA battery holder, plastic case, microphone connectors, Micro SD connector, leds,
push button, cables, resistors and capacitors.

<img src="/figures/GNS_MAIN.jpg" width="155" height="126"> <img src="/figures/GNS_cables.jpg" width="175" height="126"> <img src="/figures/GNS_mics.jpg" width="185" height="126">

## Documentation

The coding part of the present project was developed in Mbed Online Compiler (https://os.mbed.com/handbook/mbed-Compiler), platform which is already deprecated. The author has already succesfuly compiled the code in  Keil Studio Cloud (https://studio.keil.arm.com), but the output binary hasn't been tested in the hardware.

The aim of uploading this project is to port it to a new platform, probably, to increase the capabilities, the microcontroller taget would be the ESP32 from ESPRESSIF SYSTEMS.

## Citation

Please reference the project as follows:

Quintero, G., Balastegui, A., & Romeu, J. (2019). A low-cost noise measurement device for noise mapping based on mobile sampling. Measurement: Journal of the International Measurement Confederation, 148, 106894. https://doi.org/10.1016/j.measurement.2019.106894
