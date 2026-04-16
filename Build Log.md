# RC Plane Build Log
Tracking files and logbook for next features or changes


###  10/03/26:
First nrf24l01 echo prototype. Controller sends inputs to "plane" and plane will echo imu data with RTT. Controller and plane both have 0.96" oled displays for echoed input graphics and IMU readout, with controller also having RTT calculation. Controller and Plane programs are nearly identical.

_ToDo: Need to add motor control code using HW PWM for servos and motors, and programs need to be cleaned up and optimized. Considering multicore to reduce latency_

### 16/04/26:
I forgot to update the build log :(. Final version has been completed as of 14/04/26. The final program had many features cut for the sake of time, unfortunately, but is fully working aside from hardware issues. It turns out the antenna has a significantly shorter range than what was advertised, potentially due to it requiring proper alignment between the two points rather than having more degrees of freedom. The final build did have some fitment issues but is overall very solid, with all control surfaces operating smoothly and as intended. The only potential physical issue is the weight being balanced ore in front of the wings which may cause issues in our first test which has not been scheduled yet.
