# Driving_Behavior_Monitoring_System

/*  I developed a Driving Behaviour Monitoring and Alert System using two STM32 as MCUs communicating through CAN bus protocol. One MCU(slave) senses driver's acceleration, 
brake and steering data through potentiometers and sends data through CAN to the master MCU. The master MCU, working on RTOS tasks, receives data, actuates alarm on misbehaviour,
displays speed and acceleration and warnings on OLED display(I2C protocols) and transmits serially some data to ESP32(UART protocol). The ESP32 transmits it to cloud and monitored
behavior is visible on Thingsboard Dashboard. */

