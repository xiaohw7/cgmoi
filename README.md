# CGMOI User Guide

Code uses FreeRTOS. There will be 3 tasks, namely Gyro, Motor and Cg. Code will suspend Gyro and Motor task in void setup(). Cg will run at highest priority. After uploading code, Arduino will already run task Cg and read values from load cells. While running task Cg, user can control linear actuators and read values from load cells. User can send command to suspend Cg task and resume Gyro and Motor task in order to start obtaining angular acceleration values. While running Gyro and Motor task, user can also send command to suspend Gyro and Motor task and resume Cg task to return to reading values from linear actuators.

1. Obtaining Centre of Gravity:
  * While in Cg task, code outputs overall mass of satellite in grams and cartesian coordinate of Centre of Gravity in millimeters.
  

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
