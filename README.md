# CGMOI User Guide
Void loop will be an if (serial.available()) loop that waits for input from user.

User can input commands in serial monitor to control individual linear actuators, raise/lower all linear actuators, retrieve values from load cells, tare load cells, retrieve values from gyroscope while step motor spins top plate.

## Commands to send:
- Send '1' to turn on power to linear actuators

- Send '2' to turn off power to linear actuators

- Send '3'/'5'/'7' to move linear actuator 1/2/3 up for 1 second

- Send '4'/'6'/'8' to move linear actuator 1/2/3 down for 1 second

- Send '9' to move all 3 linear actuators up fully

- Send '10' to move all 3 linear actuators down fully
