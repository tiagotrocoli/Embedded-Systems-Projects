# 1. Embedded-Systems-Projects
Projects and small projects of embedded systems using STM32F407xx or TM4C123.

Bare metal programming projects involving:
- Master-slave communication using SPI, UART and I2C between STM32F407 / TM4C123xx and Arduino.
- Development of drivers of SPI, UART, I2C, ADC e etc.
- Traffic light control.
- Bootloader
- Measurement of distance using ultrasonic sensor
- Servo-motors and PWM.
- Development of drivers of STM32F407xx to communicate with LCD (HD44780)
- Measurement of the frequency and period of signal coming from Arduino to TM4C123.

FreeRTOS projects:
- Multitasking projects that use FreeRTOS, CMSIS and peripheral of STM32F407xx boards.

It's based on the book:
- ARM-Based Microcontroller Multitasking Projects Using the FreeRTOS

# 2. Description of main projects <br />

### 2.1 &nbsp; ON-OFF temperature controller <br />

<p align="center">
<img src="/images/Project_Temperature_ON_OFF.png" height="60%" width="60%">  
</p>

It is a project based on the book "ARM-Based Microcontroller Multitasking Projects: Using the FreeRTOS Multitasking Kernel". It's an academic project that uses multitasking approach to control peripheral devices using real time operating systems (FreeRTOS).

| Task         | Resume of the Function   
| -------------|------------- 
| Task 1       | It's the main function. It periodically reads the built-in temperature sensor and receives a setValue from Task 2. If the temperature is greater than the setValue, turn on the the LED. Otherwise, turn off.	If the temperature is greater than a pre-defined value, turn on the buzzer.
| Task 2       | It's the UART controller. It sends a message to the PC, and reads the input (setValue) from the PC sending it to Task 1.
| Task 3       | LCD controller. It displays in the LCD the temperature and setValue received from Task 1.
| Task 4       | It turn on/off the buzzer. Task 1 sends that request.


| Queue handle | Description   | Used by Tasks 
| -------------|---------------|-------------
| xUARTQueue   |  UART queue   | Task 1, 2	
| xLCDQueue    |  LCD  queue   | Task 1, 3
| xBuzzerQueue |  Buzzer queue | Task 1, 4


| Soft. Tools  | Function   
| -------------|------------- 
| UART4	       | It's used to communicate STM32F407 with PC
| ADC1         | It's used to measure the built-in temperature
| SPI2         | It's used to communicate STM32F407 with LCD

<p align="center">
<img src="/images/project_21.jpg" height="60%" width="60%">  
</p>
This image shows this project, however, I didn't use a buzzer module since I didn't have it. So, instead, I used a LED. The image doesn't show clearly the LCD screen, it displays "Temperature: 51" and "Setpoint: 70".

Project's file: Book_Project \ Project_21 


### 2.2 &nbsp; Distance measurement using TM4C123 and HC-SR04 ultrasonic sensor <br />

<p align="center">
<img src="/images/HC-SR04-Ultrasonic-Sensor-Pinout-diagram-768x546.jpg" height="60%" width="60%">  
</p>

Description: To find the distance from the ultrasonic sensor to an object, do the following. TM4C123 produces a 10 microsecond pulse to trigger the pin (PB4) of the sensor. Thus, the sensor produces 8 pulses of ultra-sound waves and each pulse has 40Khz of frequency. As soon as all eight pulses are transmitted through the air, the echo pin goes HIGH. In other words, the output echo pin makes transition from an active LOW to active HIGH (rising edge). The echo pin remains HIGH until the sound wave reaches the sensor after hitting an object. After the sound wave reaches the sensor, the echo pin goes LOW (falling edge). Thus, by measuring the time between rising edge and falling edge of the echo pin, it’s possible to find the distance of the sensor to an object.

<p align="center">
<img src="/images/Ultrasonic_description.png" height="80%" width="80%">  
</p>

| Soft. Tools  | Function   
| -------------|------------- 
| TIMER0       | It's a delay timer to measure the time of 10 microsecond pulses.	
| TIMER1       | It's used to calculate the time between the rising and falling edge
| UART5        | It is used to connect TM4C123 to PC sending the distance value.

| Pinout       | Function   
| -------------|------------- 	
| PB4 pin      | It connects the TRIGGER.
| PB6 pin      | It connects the ECCHO.
| PE4 pin      | UART5 RX
| PE5 pin      | UART5 TX

Project's file: Ultrasonic_sensor

### 2.3 &nbsp; FreeRTOS Queue Processing  <br />

The application has 6 commands that the user can send from PC via UART to STM32F407:

| Command            | Function   
| -------------------|-------------
|  LED_ON            | turn on a LED
|  LED_OFF           | turn off a LED
| LED_TOGGLE         | turn on toggle a LED
| LED_TOGGLE_OFF     | turn off toggle a LED
| LED_READ_STATUS    | read status of a LED
| RTC_PRINT_DATETIME | print datetime

under construction...

### 2.4 &nbsp; Frequency Measurement by TM4C123 of signal comming from Arduino <br />

<p align="center">
<img src="/images/frequency_measrement.png" height="80%" width="80%">  
</p>

In this small project the arduino sends a signal to TM4C123 and it calculate the frequency of it sending it to PV via UART. To do, so it measure the time difference 
between the rising and falling edge using TIMER0 module, that is the period. The frequecy is just the inverse of the period.


| Soft. Tools  | Function   
| -------------|------------- 	
| TIMER0       | It's used to calculate the time between rising and falling edge
| UART5        | It is used to connect TM4C123 to the PC.

| Pinout       | Function   
| -------------|------------- 	
| PE4 pin      | UART5 RX
| PE5 pin      | UART5 TX

File: Frequency_measurement

It's based on courses:
- Master microcontroller with embedded drivers.
- Master microcontroller: TIMERS, PMW, CAN, RTC, LOW POWER.
- Embedded system programming on ARM Cortex M3/M4 processor.
- Embedded Systems - Shape The World: Microcontroller Input/Output
- Mastering RTOS: Hands on FreeRTOS and STM32Fx with Debugging
- Complete ARM Cortex-M Bare-Metal Programming Ground Up



