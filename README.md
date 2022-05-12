# CGMOI User Guide

## Overview

![cgmoi machine](https://github.com/xiaohw7/cgmoi/blob/main/Images/cgmoi_machine.jpg)

**Machine is currently still a work in progress.**

The cgmoi machine is one that is capable of measuring the Centre of Gravity (CG) and Moment Of Inertia (MOI) of a nano-satellite. The concept of this machine is based on [this research paper](https://github.com/xiaohw7/cgmoi/blob/main/applsci-08-00104.pdf) done by some guys from ZheJiang University. It uses 3 load cells in a circle spaced 120 degrees apart to measure CG and an accelerometer to measure acceleration which in turn can be used to obtain MOI.

The machine also includes a stepper motor that spins the top plate, and linear actuators to raise and lower the load cells in order to measure CG.

The stepper motor, linear actuator, accelerometer, and load cells are controlled by and wired to an Arduino mega board. The Arduino could then be plugged into a laptop running the Arduino IDE where code could be uploaded to run the cgmoi machine.



## How to use

1. User would have to download the following Arduino libraries:
      - [HX711_ADC](https://github.com/olkal/HX711_ADC) for the load cells
      - [hanyazou](https://github.com/hanyazou/BMI160-Arduino) for the BMI160 gyroscope
      - [AccelStepper](https://github.com/waspinator/AccelStepper) for stepper motor and stepper motor driver
      - [FreeRTOS](https://github.com/feilipu/Arduino_FreeRTOS_Library) for running Real Time Operating System on Arduino
      - [SimpleKalmanFilter](https://github.com/denyssene/SimpleKalmanFilter) for running values through a Kalman filter to reduce noisy readings

2. Upload cgmoi.ino onto Arduino and open Serial monitor.

3. Code uses FreeRTOS. There will be 3 tasks, namely Gyro, Motor and Cg. Code will suspend Gyro and Motor task immediately in void setup() and Cg task will run at highest priority.

4. After uploading code, Arduino will be running task Cg and user can see output values from load cells on Serial monitor. User can either continue with Cg task to measure CG of satellite or choose to measure MOI of satellite instead.

5. If user intends to measure CG of satellite:

      * Linear actuators must be fully extended.

      * ![screw securing removeable shaft](https://github.com/xiaohw7/cgmoi/blob/main/Images/screw_securing_removeable_shaft.JPG)

      * Ensure screw securing removeable shaft (shown above) is removed before sending '9' in Serial monitor to raise linear actuators completely. Then, send '11' in serial monitor to tare load cells before mounting satellite on top plate.

      * While running task Cg, mass and CG coordinates can be read straight off the output on the Serial monitor. Refer to "Calculations" section below for x and y axis. Coordinates are in mm.

6. If user intends on measuring MOI of satellite:

      * Linear actuators must be fully retracted. Send '10' to fully retract linear actuators.

      * Ensure screw securing removeable shaft is secured, send '14' to suspend Cg task and start Gyro and Motor tasks.

      * Gyro and Motor tasks will run simultaneously and angular acceleration values can be read from serial monitor. Motor will spin back and forth to allow gyro to measure acceleration values. Once measurement process is finished, average acceleration will be displayed on serial monitor along with "Finish recording" message. User has 10 seconds before recording process starts again.

      * User should record down angular acceleration values with and without satellite mounted. Refer to "Calculations" section for the equations to obtain MOI.

7. While running Gyro and Motor task, user can also send command to suspend Gyro and Motor task and resume Cg task to return to reading values from linear actuators.

## Commands to send

|Task |Commands that can be sent |What it does|
|-----|--------------------------|------------|
|Cg |Send '1' |Turn on power to linear actuators|
|Cg |Send '2' |Turn off power to linear actuators|
|Cg |Send '3'/'5'/'7' |Move linear actuator 1/2/3 up for 1 second|
|Cg |Send '4'/'6'/'8' |Move linear actuator 1/2/3 down for 1 second|
|Cg |Send '9' |Move all 3 linear actuators up fully|
|Cg |Send '10' |Move all 3 linear actuators down fully|
|Cg |Send '11' |Tare all load cells|
|Cg |Send '12' |Turn on power to stepper motor|
|Cg |Send '13' |Turn off power to stepper motor|
|Cg |Send '14' |End Cg task and resume/start Gyro and Motor tasks|
|Gyro and Motor |Send '12' |Turn on power to stepper motor|
|Gyro and Motor |Send '13' |Turn off power to stepper motor|
|Gyro and Motor |Send '15' |End Gyro and Motor tasks and resume Cg task|


## Points to take note

- Before raising/lowering linear actuators, user must remember to remove screw securing removeable shaft is removed.

- When lowering top plate, user should stand by and make sure removeable shaft enters designated slot correctly.

- Before measuring MOI, ensure screw securing removeable shaft is tightened to prevent any play when motor is turning.

- Before measuring MOI, it is recommended to position gyro chip above red tape. User can rotate top plate a set number of steps using Stepper_motor_driver2.ino.

- There is a tendency for SPDT relay supplying power to stepper motor to get stuck in the close position despite the LED light being off and signal sent to it to disconnect. This may be because relay is only rated for 30V while power supply is at 36V. Tapping the blue box on the relay would help to disconnect it. User can tell if power had been disconnected by observing light on stepper motor driver.

- Angular acceleration values is rather unreliable and difference while loaded and unloaded may not be easily observed. Sometimes loaded values may even be higher than unloaded values. May be due to looseness and play at the epoxy region of removeable shaft.

- 

- If load cell readings become inaccurate, try running "Calibration" example and obtain calibration values for each individual load cell and input into the code.

- If there is a need to stop measurements/linear actuators/stepper motor immediately, unplug Arduino to "Emergency Stop".



## Calculations

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

3. Finding Moment Of Inertia

      * ![moi_eqn](https://github.com/xiaohw7/cgmoi/blob/main/Images/moi_eqn.png)

      * Use equation above to obtain J1. Where J1 is MOI of satellite. J0 is MOI of fixture tools which can be obtained from solid works.
      *  w0 is angular velocity at time t without satellite. w’0 is angular velocity at time t with satellite. t1 and t2 are time at which angular velocity w0 and w’0 is taken respectively.
      * Thus it is observed that the fraction in the equation is angular acceleration with satellite mounted divided by angular acceleration without satellite mounted.
      * Code outputs angular acceleration values in degrees/sec^2 which can then be used to calculate J0.

      * Rotating about X-axis of satellite will result in JX

      * Rotating about Y-axis of satellite will result in JY

      * Rotating about Z-axis of satellite will result in JZ

      * ![parallel axis theorem](https://github.com/xiaohw7/cgmoi/blob/main/Images/parallel_axis_theorem.png)

      * Above equation uses parallel axis parallel axis theorem to correct MOI vector with reference to the CG and find vector of MOI. JxG, JyG, JzG are the MOI in x, y, and z direction respectively.

## Instructions
(Below are instructions on how to set up each individual component of the cgmoi machine as well as some notes I made on the problems I faced)

### HX711 ADC and load cells:

- Refer to [this link](https://makersportal.com/blog/2019/5/12/arduino-weighing-scale-with-load-cell-and-hx711) for instructions on how to connect load cell to HX711 to Arduino. All 3 load cells can share a common power and ground connection through a breadboard.

- Download library at [github](https://github.com/olkal/HX711_ADC) for examples on calibrating and reading values from multiple load cells.

- To calibrate load cell, connect Arduino to an individual load cell through HX711 and run “Calibration” example. Follow instructions on the serial monitor and record down “calibration value”.  Do this for every individual load cell.

- Use `HX711_script3.ino` to read all 3 load cells. Input “calibration values” of each respective load cell into the code. Weight reading is taken 10 times with 0.05sec intervals and average weight on each load cell is obtained. Calculations are then made with the average weights. Run the code and read values of weight and coordinates of CG from serial monitor.

- Send “t” on serial monitor to tare load cells immediately.

- Reading is in grams. CG units are in mm.

### BMI 160 Gyro:

- Refer to [hanyazou's github](https://github.com/hanyazou/BMI160-Arduino) for instructions to connect to gyro sensor. Download library at the same website. Use example `Gyro` example or `bmi_160_gyro_script.ino` to read from sensor. Switch example code to I2C mode.

- Another library that can read from gyro is [DFRobot](https://github.com/DFRobot/DFRobot_BMI160#installation). Download the library and use `accelgyro` example for both accelerometer and angular velocity readings.

- `bmi_160_gryo_script2.ino` takes `accelgyro` example from DFRobot library and modifies it to obtain net acceleration from x and y axis using Pythagoras theorem. Measurements are taken every 0.1 seconds and the average of 10 readings are calculated. Acceleration is converted from g to degrees/sec^2.

- `bmi_160_gryo_script3.ino` modifies `bmi_160_gyro_script.ino` to get net acceleration using Pythagoras theorem on the x an y axis acceleration values.

- Net acceleration is then divided by radius (distance from gyro to center of top plate) to obtain angular acceleration in radians which is then converted to degrees.

- Sensor is to use 3.3v from Arduino.

- Helpful links: (http://www.arduinoprojects.net/sensor-projects/using-bmi160-sensor-arduino-uno.php) , https://learn.sparkfun.com/tutorials/gyroscope/all

### Linear actuator with relay:

- One SPDT relay to be used as overall power switch to linear actuators. 3 DPDT relays to be used as switches for each individual actuator and to be connected in a way to allow current to flow in both direction so as to enable actuator to extend and retract.

- Helpful links for SPDT relay can be found at [circuit basics](https://www.circuitbasics.com/setting-up-a-5v-relay-on-the-arduino/) and [last minute engineers](https://lastminuteengineers.com/one-channel-relay-module-arduino-tutorial/).

- Wiring for DPDT relay to be able to reverse current polarity to actuators can be found at [digikey](https://forum.digikey.com/t/polarity-reversal-using-a-dpdt-switch/626/4).

- Wiring:

![linear actuator wiring](https://github.com/xiaohw7/cgmoi/blob/main/Images/linear_actuator_wiring.png)

- Above picture shows wiring of linear actuators with relays.

- VCC and GND of all relays are connected to 5V and GND of Arduino through breadboard.

- For each DPDT relay, NO2 and NC1 should be connected to the black wire of actuator while NO1 and NC2 should be connected to the red wire of actuator.

- Use `linear_actuator_script.ino` to control the actuators through serial monitor.

- Send ‘1’ to power on. Send ‘2’ to power off.  

- Send ‘3’, ‘5’, or ‘7’ to move actuator 1, 2, or 3 up for 1 sec respectively.

- Send ‘4’, ‘6’, or ‘8’ to move actuator 1, 2, or 3 down for 1 sec respectively.

- Send ‘9’ to move all 3 actuators up completely.

- Send ‘10’ to move all 3 actuators down completely.

- Linear actuator to use 12V from power supply.

### LICHUAN LCDA257S stepper motor driver and stepper motor:

- See [dronebotworkship](https://dronebotworkshop.com/big-stepper-motors/) to get an overview on stepper drivers and motors.

- See [makersguides](https://www.makerguides.com/tb6600-stepper-motor-driver-arduino-tutorial/) for example code and wiring.

- Number of pulses per round can be adjusted by referring to table on the driver.

- Download AccelStepper library from [github](https://github.com/waspinator/AccelStepper).

- In AccelStepper library, speed is in steps/sec, Acceleration is in steps/sec2, position is in number of steps.

- `Stepper_motor_driver.ino` controls stepper motor directly while `Stepper_motor_driver2.ino` uses AccelStepper library to control motor.

- Another way of accelerating stepper motor is do gradually quicken pulses sent to motor (ramping). Code is in `Stepper_motor_driver3.ino`.

- Stepper motor to use 36V from power supply.

### Putting it together (Gyro and Motor):

- Function to control movement of stepper motor in AccelStepper library is blocking, thus we use FreeRTOS on Arduino, Gyro and Motor can run and read data simultaneously. FreeRtos library can be found at [github](https://github.com/feilipu/Arduino_FreeRTOS_Library).

- `moi.ino`(`Stepper_motor_driver2.ino` + `bmi_160_gryo_script3.ino`) uses FreeRTOS and AccelStepper library to run motor and gyro simultaneously.

- Alternatively, we can use Pulse Width Modulation to send pulses to stepper motor. analogWrite() function output a PWM signal to stepper motor and is not a blocking function. Hence can collect data from gyro immediately after starting stepper motor.

- Code to toggle PWM frequencies in Arduino can be found  [here](https://www.etechnophiles.com/how-to-change-pwm-frequency-of-arduino-mega/).

- Take note: code to change PWM frequency to 62500 Hz changes delay() function such that delay(1000000) now is approximately 15 sec.

- Also, offset values are recorded so as to account for gyro not being completely flat. Code can be found in `moi2.ino` (using PWM to control stepper motors + `bmi_160_gryo_script2.ino`).

- However, Gyro still experiences large amount of noise. To reduce noise, gyro code include a digital low pass filter in the code from [hanyazou's github](https://github.com/hanyazou/BMI160-Arduino).  Also, a simple Kalman Filter found on [github] (https://github.com/denyssene/SimpleKalmanFilter) is added into the code which decreased standard variation when top plate is stationary as well as when it is moving, the final angular acceleration value is put through the filter.  

- Using FreeRTOS and PWM with altered frequency of 31372.55 Hz, a graph of angular acceleration against time is plotted through the acceleration and deceleration of the stepper motor.  It is observed that there is a very sharp peak acceleration when stepper motor accelerates and acceleration. Hence the gyro will measure acceleration values throughout the motor’s acceleration and the code will record down the peak acceleration value.

- Accelerometer sample rate is changed from 200Hz to 1600Hz so as to increase accuracy. Peak acceleration values are put in an array and after a certain number or recordings, mean and standard deviation are calculated from the peak values in array. Code is in `moi3.ino` (using PWM to control stepper motors + `bmi_160_gryo_script3.ino`).

- However, peak acceleration values tend to vary substantially (sometimes up to 60%). We can use FreeRTOS to run gyro and motor with AccelStepper library simultaneously. Motor will accelerate at a set acceleration and gyro can measure average acceleration during that time.

- 3 tasks are created, namely Control, Motor and Gyro. Control task will run with highest priority and will first suspend Gyro and Motor task. Purpose of Control task is to take commands from serial monitor and start Gyro and Motor task to begin recording of values.

- While motor task is causing motor to accelerate, Gyroscope task will run and read acceleration values with each loop. Average acceleration is obtained by dividing total acceleration by number of loops variable loopCount. Average acceleration values are added to an array and a subsequent average is obtained at the end of a set number of recordings.

- When switching from Control task to Gyro and Motor task, first reading is buggy and inaccurate and hence not added to the array. Only 2nd reading onwards are added to the array. loopCount can also be used to limit gyro’s recording to the acceleration process of the motor and not both acceleration and deceleration.

- A new problem arise when load is put on top plate, average acceleration is observed to increase instead of decrease due to majority of acceleration values becoming higher despite peak acceleration becoming lower. This may be due to the usage of AccelStepper library which allows motor to draw more current to maintain set acceleration. Solution is to max out setAcceleration() and setMaxSpeed() functions so that motor could not draw any additional current and to set dip switches on the side of the stepper motor driver to 1600 pulses/round. Code is in `moi4.ino` (modified from `moi.ino`).

### Putting it together (Linear Actuator and Load Cells):
- Refer to above instructions “Linear actuator with relay” for instructions on how to operate linear actuators. Once Arduino is on, load cells are already reading weight values and calculating CG.

- Linear actuators should be tested first by moving each actuator up and down.

- Linear actuators should be extended all the way to measure weight of satellite. Satellite should be placed on the top plate after actuators fully extend and taring load cells. Wait for “Tare load cell 1 / 2 / 3 complete” message. User should also tare load cells after each measurement.

- Arduino mega is able to support all 3 load cells and the 4 relays hence all 7 components will share the same VCC and GND through breadboard. Code is in `cg.ino` (`linear_actuator_script.ino` + `HX711_script3.ino`).

### Putting it together (CG and MOI):

- Code uses FreeRTOS. There will be 3 tasks, namely Gyro, Motor and Cg. Code will suspend Gyro and Motor task in void setup(). Cg will run at highest priority.

- While running task Cg, user can control linear actuators and read values from load cells. User can send command to suspend Cg task and resume Gyro and Motor task in order to start obtaining angular acceleration values. While running Gyro and Motor task, user can also send command to suspend Gyro and Motor task and resume Cg task to return to reading values from linear actuators.

- Code is in `cgmoi.ino` (`moi4.ino` + `cg.ino`).
