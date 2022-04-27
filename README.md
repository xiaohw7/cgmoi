# CGMOI User Guide

Code uses FreeRTOS. There will be 3 tasks, namely Gyro, Motor and Cg. Code will suspend Gyro and Motor task in void setup(). Cg will run at highest priority. After uploading code, Arduino will already run task Cg and read values from load cells. While running task Cg, user can control linear actuators and read values from load cells. User can send command to suspend Cg task and resume Gyro and Motor task in order to start obtaining angular acceleration values. While running Gyro and Motor task, user can also send command to suspend Gyro and Motor task and resume Cg task to return to reading values from linear actuators.

## Commands to send

Commands to send while running Cg task:

- Send '1' to turn on power to linear actuators

- Send '2' to turn off power to linear actuators

- Send '3'/'5'/'7' to move linear actuator 1/2/3 up for 1 second

- Send '4'/'6'/'8' to move linear actuator 1/2/3 down for 1 second

- Send '9' to move all 3 linear actuators up fully

- Send '10' to move all 3 linear actuators down fully

- Send '11' to tare all load cells

- Send '12' to turn on power to stepper motor

- Send '13' to off power to stepper motor

- Send '14' to end Cg task and resume/start Gyro and Motor tasks

Commands to send while running Gyro and Motor tasks:

- Send '12' to turn on power to stepper motor

- Send '13' to off power to stepper motor

- Send '15' to end Gyro and Motor tasks and resume Cg task

### Calculations

![coordinates](https://github.com/xiaohw7/cgmoi/blob/main/Images/coordinates%20cgmoi.png)

With reference to image above, points A,B,C correspond to load cell 1,2,3 respectively.

1. Finding Mass
  * Pa = m4 - m1
  * Pc = m6 - m3
  * Pb = m5 - m2
  * Mass of satellite, M = Pa + Pb + Pc  
  * m1, m2 and m3 are weights of fixture tools and platform on load cell A, A and A respectively.
  * m4, m5 and m6 are wights of fixture tools, platform and satellite on load cell A, B and C respectively.
  * Pa, Pb and Pc are satellite weights on load cell a, b and c respectively.
  * Code outputs overall mass of satellite in grams.

2. Finding Centre of Gravity
  * Y-coordinate, yG = [Pb(y2) + Pc(y3) − Pa(y1)]/M
  * X-coordinate, xG = [Pb(x1) − Pc(x2)]/M
  * y1, y2, y3 are distances from x-axis. y1 = 206mm, y2 and y3 = 103mm
  * x1, x2, are distances from y-axis. x1 and x2 = 178.4mm
  * Lengths are obtained from datasheets in OneDrive and calculations based on the information in datasheets.
  * To obtain z-coordinate of CG, rotate satellite by 90 degrees and undergo same calculations to obtain z-coordinate.
  * Code outputs coordinate of CG with reference to axis above and in millimeters.
  * Note: resultant x-axis values from calculations are opposite to the above axis shown in image. If location of CG moves upwards closer position B, x coordinate of CG becomes more positive.

2. Finding Moment Of Inertia

![moi_eqn](https://github.com/xiaohw7/cgmoi/blob/main/Images/moi_eqn.png)
  * Use equation above to obtain J1. Where J1 is MOI of satellite. J0 is MOI of fixture tools which can be obtained from solid works.
  *  w0 is angular velocity at time t without satellite. w’0 is angular velocity at time t with satellite. t1 and t2 are time at which angular velocity w0 and w’0 is taken respectively.
  * Thus it is observed that the fraction in the equation is angular acceleration with satellite mounted divided by angular acceleration without satellite mounted.
  * Code outputs angular acceleration values in degrees/sec^2 which can then be used to calculate J0.

  * Rotating about X-axis of satellite will result in JX

  * Rotating about Y-axis of satellite will result in JY

  * Rotating about Z-axis of satellite will result in JZ

![parallel axis theorem](https://github.com/xiaohw7/cgmoi/blob/main/Images/parallel_axis_theorem.png)
  * Above equation uses parallel axis parallel axis theorem to correct MOI vector with reference to the CG and find vector of MOI. JxG, JyG, JzG are the MOI in x, y, and z direction respectively.

#### Instructions
(Below are instructions on how to set up each component of the cgmoi machine as well as some notes I made on the problems I faced)

HX711 load cell:

- Refer to [this link](https://makersportal.com/blog/2019/5/12/arduino-weighing-scale-with-load-cell-and-hx711) for instructions on how to connect load cell to HX711 to Arduino. All 3 load cells can share a common power and ground connection through a breadboard.

- Download library at [github](https://github.com/olkal/HX711_ADC) for examples on calibrating and reading values from multiple load cells.

- To calibrate load cell, connect Arduino to an individual load cell through HX711 and run “Calibration” example. Follow instructions on the serial monitor and record down “calibration value”.  Do this for every individual load cell.

- To calibrate load cell, connect Arduino to an individual load cell through HX711 and run “Calibration” example. Follow instructions on the serial monitor and record down “calibration value”.  Do this for every individual load cell.

- Use HX711_script3.ino to read all 3 load cells. Input “calibration values” of each respective load cell into the code. Weight reading is taken 10 times with 0.05sec intervals and average weight on each load cell is obtained. Calculations are then made with the average weights. Run the code and read values of weight and coordinates of CG from serial monitor.

- Send “t” on serial monitor to tare load cells immediately.

- Reading is in grams. CG units are in mm.

BMI 160 Gyro:

- Refer to [hanyazou's github](https://github.com/hanyazou/BMI160-Arduino) for instructions to connect to gyro sensor. Download library at the same website. Use example “Gyro” example or bmi_160_gyro_script.ino to read from sensor. Switch example code to I2C mode.

- Another library that can read from gyro is [DFRobot](https://github.com/DFRobot/DFRobot_BMI160#installation). Download the library and use accelgyro example for both accelerometer and angular velocity readings.

- bmi_160_gryo_script2.ino takes accelgyro example from DFRobot library and modifies it to obtain net acceleration from x and y axis using Pythagoras theorem. Measurements are taken every 0.1 seconds and the average of 10 readings are calculated. Acceleration is converted from g to degrees/sec^2.

- bmi_160_gryo_script3.ino modifies bmi_160_gyro_script.ino to get net acceleration using Pythagoras theorem on the x an y axis acceleration values.

- Helpful links: http://www.arduinoprojects.net/sensor-projects/using-bmi160-sensor-arduino-uno.php , https://learn.sparkfun.com/tutorials/gyroscope/all

Linear actuator with relay:

- 1 SPDT relay to be used as overall power switch to linear actuators. 3 DPDT relays to be used as switches for each individual actuator and to be connected in a way to allow current to flow in both direction so as to enable actuator to extend and retract.

- Helpful links for SPDT relay can be found at [circuit basics](https://www.circuitbasics.com/setting-up-a-5v-relay-on-the-arduino/) and [last minute engineers](https://lastminuteengineers.com/one-channel-relay-module-arduino-tutorial/).

- Wiring for DPDT relay to be able to reverse current polarity to actuators can be found at [digikey](https://forum.digikey.com/t/polarity-reversal-using-a-dpdt-switch/626/4).
