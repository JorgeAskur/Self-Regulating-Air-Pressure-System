# Self Regulating Air Pressure System
## 5th semester project

Traveling through large distances is sometimes required by truck owners to move their tractors from a storage room to the farm, where the truck’s tires are affected by the multiple types of terrain in which the automobile has to go, as well as the need to complete different tasks in which the vehicle must accommodate a certain height. 
This is why our job is to establish an air control system that regulates the pressure of the tires when the tractor is at different terrains, which would allow the truck to set a different height that would help the system to be less damaged.

This project was developed for the Advanced Embedded Systems Design class in colaboration with John Deere. 

The solution to this problem should satisfy the following goals:

* The construction of a working air pressure tank
* Regulation of the air pressure depending on the terrain. 
* Prevention of overfilling the tank.

The main purpose of this system is to pump air pressure into a PVC tank, simulating the tire of a tractor, so it can adjust to terrains such as asphalt, mud and dirt. This would be useful for the tractor’s performance when it encounters these different lands and cause less damage to the tractor itself. 

In this project we used:

* CAN protocol
* ADC interfacing
* WiFi communication via ESP8266
* Digital Signal Processing
* UART communication
* Peripheral Interfacing via PWM & GPIO
* PID Control

#List of Materials

* STM32H745ZI
* NodeMCU
* CAN Transceiver TJA1050
* Arduino CAN Shield
* Moto Monster Shield
* Relay
* Pressure Sensor
* Air Pump
* Solenoid Valve

#Schematic
![imagen](https://user-images.githubusercontent.com/74482029/206890135-b0d688f3-5255-453f-b7fb-229f076b0f55.png)



## Team members
* Jorge Askur Vazquez
* Andrés Sarellano
* Izel María Ávila
