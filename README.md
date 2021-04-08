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

<p align="center">
<img src="/images/project_21.jpg" height="60%" width="60%">  
</p>
This image shows this project, however, I didn't use a buzzer module since I didn't have it. So, instead I used a LED. 

Project's file: Book_Project \ Project_21 


### 2.2 &nbsp; Distance measurement using TM4C123 and HC-SR04 ultrasonic sensor <br />

<p align="center">
<img src="/images/HC-SR04-Ultrasonic-Sensor-Pinout-diagram-768x546.jpg" height="60%" width="60%">  
</p>

Description: To find the distance from the ultrasonic sensor to an object, do the following. TM4C123 produces a 10 microsecond pulse to trigger the pin (PB4) of the sensor. Thus, the sensor produces 8 pulses of ulta-sound waves and each pulse has 40Khz of frequency. As soon as all eight pulses are transmitted through the air, the echo pin goes HIGH. In other words, the output echo pin makes transition from an active LOW to active HIGH (rising edge). The echo pin remains HIGH until the sound wave reaches the sensor after hitting an object. After the sound wave reaches the sensor, the echo pin goes LOW (falling edge). Thus, by measuring the time between rising edge and falling edge of the echo pin, it’s possible to find the distance of the sensor to an object.

| Soft. Tools  | Function   
| -------------|------------- 
| TIMER0       | It's a delay timer to measure the time of 10 microsecond pulses.	
| TIMER1       | It's used to calculate the time between the rising and falling edge
| PB4 pin      | It connects the TRIGGER.
| PB6 pin      | It connects the ECCHO.

Project's file: Ultrasonic_sensor

### 2.3 &nbsp; FreeRTOS Queue Processing  <br />

Description under construction

### 2.3 &nbsp; Frequency Measurement by TM4C123 of signal comming from Arduino <br />

Description under construction


It's based on courses:
- Master microcontroller with embedded drivers.
- Master microcontroller: TIMERS, PMW, CAN, RTC, LOW POWER.
- Embedded system programming on ARM Cortex M3/M4 processor.
- Embedded Systems - Shape The World: Microcontroller Input/Output
- Mastering RTOS: Hands on FreeRTOS and STM32Fx with Debugging
- Complete ARM Cortex-M Bare-Metal Programming Ground Up



