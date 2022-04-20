# CGMOI User Guide
Program runs using FreeRtos where a task with highest priority will be able to control other tasks such as Gyro and Motor or linear actuators or load cells by suspending and resuming tasks.

User can input commands in serial monitor to resume/suspend tasks, control individual linear actuators, raise/lower all linear actuators, retrieve values from load cells, tare load cells, retrieve values from gyroscope while step motor spins top plate.

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
