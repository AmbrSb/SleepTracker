[ Ongoing Project ]
# SleepTracker
# Description
ESP8266, Arduino, and Raspberry Pi-based realtime motion tracking for collection of body movements during sleep.
The goal of this project is collection of data from body-worn sensing units to:
- create a large dataset of body movements during sleep for use by researchers
- make a low-cost system for detection and classification of sleep stages
# Architecture
I will implement this project via two different architectures:
1. Based on Arduino nano (and LilyPad) + nRF24L01 radio modules for sensors, and also a Raspberry Pi + nRF24L01 radio on the receive side.

![image](https://user-images.githubusercontent.com/19773760/111999611-c7b7af00-8b3a-11eb-8da0-df6b4fac541f.png)

3. Based on ESP8266 module which has on chip integrated WiFi through which we can share data over a TCP/IP network.

![image](https://user-images.githubusercontent.com/19773760/112190237-708d0980-8c22-11eb-887d-5dc1fd5a91b1.png)

# Location of sensors
- Two wrists
- Two ankles
# Electronics
## Sensors
- 3-Axis Accelerometer (ADXL345) operated on ranges of +/-4g and +/-16g
- SODIAL(R) Pulse Sensor
## Collector Schematic
![image](https://user-images.githubusercontent.com/19773760/112004326-4d3d5e00-8b3f-11eb-817e-b1bd86aa7bb6.png)

## Dataset Attributes
Age, Gender, Weight, Height
Timestamp, HR
Right Wrist AccX-4g, AccY-4g, AccZ-4g, AccX-16g, AccY-16g, AccZ-16g
Left Wrist  AccX-4g, AccY-4g, AccZ-4g, AccX-16g, AccY-16g, AccZ-16g
Right Ankle AccX-4g, AccY-4g, AccZ-4g, AccX-16g, AccY-16g, AccZ-16g
Left Ankle  AccX-4g, AccY-4g, AccZ-4g, AccX-16g, AccY-16g, AccZ-16g
## Main Electronic Components
- Arduino Nano
- LilyPad Arduino
- Raspberry Pi 3 Model B
- Accelerometer (ADXL345)
- SODIAL(R) Pulse Sensor Heart Rate Sensor Heart Beat PulseSensor for Arduino Raspberry 
- nRF24L01 Single Chip 2.4 GHz Radio Transceiver
- WINGONEER XL4016E1 DC Voltage Regulator
- 9V/1A AC/DC power adaptor
- CJMCU Mini USB Power Module
- MPU9250 SPI/I2C 9-Axis Gyro Accelerator Magnetometer Module
- TinyRTC module based on DS1307 for clock + CR1225 battery
