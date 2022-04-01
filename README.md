# STM32F3 controller for SRT150 actuators

**As for now it is recommanded to use the branch ***single_timer***.**

This project allows to control up to 4 SRT150 actuators via a STM32F3-DISCOVERY board plugged to a PC in USB.

It is configured to work with SRT150 actuators which have 20cm of range (which corresponds to 20 turn of the motor). The drivers have to be configured to 3200 micro steps by turn which result in a total of 64000 micro steps for the full range (this value have to be used to configure FlyPT Mover profile).
