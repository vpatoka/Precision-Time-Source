# Precision-Time-Source
Precision Time Source / Generator based on STM32 MCU + TBolt + AD9852 and FREERTOS


Famous Trimble T-Bolt GPSDO is the heart of this project. Its 1PPS output has connected to TAPR distribution board. So, 3 TAPR outputs is using for 1PPS signal distribution. 10Mhz output has connected to LC6954 splitter. So, I have two 10MHZ signals. One of them has connected to TAPR (another three outputs) and the second 10MHZ has connected to AD9852 as a ref. signal source. Then STM32 MCU is using to manage the DDS generator (from 0Hz to 80 Mhz). Also, MCU is using to catch the GPS Trimble protocol output from TBolt RS232 port. The MCU transform the proprietary messages to human readable format and send it to LCD. In addition, MAX11205 is incorporated to measure the phase difference of two signals.

RTOS is making all this things working realtime

