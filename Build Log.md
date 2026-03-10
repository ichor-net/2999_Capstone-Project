# RC Plane Build Log
Tracking files and log book for next features or changes


###  10/03/26:
First nrf24l01 echo prototype. Controller sends inputs to "plane" and plane will echo imu data with RTT. Controller and plane both have 0.96" oled displays for echoed input graphics and IMU readout, with controller also having RTT calculation. Controller and Plane programs are nearly identical.

_ToDo: Need to add motor control code using HW PWM for servos and motors, and programs need to be cleaned up and optimized. Considering multicore to reduce latency_
