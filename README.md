# CGMOI User Guide
Program runs using FreeRtos. Program starts with CG task, which will run with highest priority, where user can send commands to raise/lower linear actuators and read values from load cells. User can also send commands to suspend CG task and to start Gyro and Motor task to measure MOI. From there, user can send command to suspend/resume Gyro and Motor task or resume CG task and go back to reading values from load cell.

## Commands to send:
- Send '1' to turn on power to linear actuators

- Send '2' to turn off power to linear actuators

- Send '3'/'5'/'7' to move linear actuator 1/2/3 up for 1 second

- Send '4'/'6'/'8' to move linear actuator 1/2/3 down for 1 second

- Send '9' to move all 3 linear actuators up fully

- Send '10' to move all 3 linear actuators down fully

- Send '11' to tare all load cells

- Send '12' to turn on power to stepper motor

- Send '13' to off power to stepper motor

- Send '14' to resume/start Gyro and Motor task
