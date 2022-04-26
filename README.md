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
