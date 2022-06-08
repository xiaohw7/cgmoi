# CGMOI User Guide

## Overview

![cgmoi machine](https://github.com/xiaohw7/cgmoi/blob/main/Images/cgmoi%20machine%20annotated.png)

**Machine is currently still a work in progress.**

The cgmoi machine is one that is capable of measuring the Centre of Gravity (CG) and Moment Of Inertia (MOI) of a nano-satellite while satellite is mounted on the top plate as shown in picture below.

![satellite mounted](https://github.com/xiaohw7/cgmoi/blob/main/Images/satellite%20mounted%20annotated.png)

The concept of this machine is based on [this research paper](https://github.com/xiaohw7/cgmoi/blob/main/applsci-08-00104.pdf) done by some guys from ZheJiang University. It uses 3 load cells in a circle spaced 120 degrees apart to measure CG and a gyroscope to measure acceleration which in turn can be used to obtain MOI.

The machine also includes a stepper motor that spins the top plate, and linear actuators to raise and lower the load cells in order to measure CG.

The stepper motor, linear actuator, gyroscope, load cells and other components are controlled by and wired to an Arduino mega board. The Arduino could then be plugged into a laptop running the Arduino IDE where code could be uploaded to run the cgmoi machine.

Each load cell is marked load cell 1/2/3 on the load cell itself. Linear actuator 1/2/3 correspond to load cell 1/2/3. i.e. linear actuator 1 is directly below load cell 1.

## Components of the cgmoi Machine

- HX711 analog to digital converter
![ADC](https://github.com/xiaohw7/cgmoi/blob/main/Images/ADC.JPG)

- Single pole double throw(SPDT) relay
![spdt](https://github.com/xiaohw7/cgmoi/blob/main/Images/SPDT.JPG)

- Double pole double throw(DPDT) relay
![dpdt](https://github.com/xiaohw7/cgmoi/blob/main/Images/DPDT.JPG)

- Gyroscope
![gyro](https://github.com/xiaohw7/cgmoi/blob/main/Images/Gyroscope.JPG)

## How to use

1. User would have to download the following Arduino libraries:
      - [HX711_ADC](https://github.com/olkal/HX711_ADC) for the load cells
      - [hanyazou](https://github.com/hanyazou/BMI160-Arduino) for the BMI160 gyroscope
      - [AccelStepper](https://github.com/waspinator/AccelStepper) for stepper motor and stepper motor driver
      - [FreeRTOS](https://github.com/feilipu/Arduino_FreeRTOS_Library) for running Real Time Operating System on Arduino
      - [SimpleKalmanFilter](https://github.com/denyssene/SimpleKalmanFilter) for running values through a Kalman filter to reduce noisy readings

2. Upload cgmoi.ino onto Arduino and open Serial monitor.

3. Code uses FreeRTOS. There will be 3 tasks, namely Gyro, Motor and Cg. Code will suspend Gyro and Motor task immediately in void setup() and Cg task will run at highest priority.

4. Code has 2 modes. Mode 1 runs Cg task, reads values from load cells and output CG and mass values. Mode 2 runs Motor and Gyro tasks and outputs angular acceleration values.

5. After uploading code, Arduino will be running mode 1 and user can see output values from load cells on Serial monitor. User can either continue with mode 1 to measure CG of satellite or switch to mode 2 to measure MOI of satellite instead.

6. Mode 1: User intends to measure CG of satellite

      * Linear actuators must be fully extended.

      * It is advisable to test each linear actuator by moving them up and down individually for 1 second before raising all three actuators completely. Refer to "Commands to send" table below for commands to move actuators individually.

      * Ensure ***screw securing removeable shaft (shown below) is removed*** before sending '9' in Serial monitor to raise linear actuators completely. Then, send '11' in serial monitor to tare load cells before mounting satellite.

      * ![screw securing removeable shaft](https://github.com/xiaohw7/cgmoi/blob/main/Images/screw_securing_removeable_shaft.JPG)

      * Directions of x and y axis are indicated on the top plate and also illustrated in "Calculations" section below.

      * To ease measuring process of CG, x and y axis on top plate must align with axis indicated in "Calculations" section below. Arrow of y axis must point directly away from load cell 1. Location of origin is the center of the top plate.

      * One way to align x and y axis is to align the marked line on the edge of the top plate with the number 1 indicated on load cell 1. An illustration is shown below.

      * ![axis alignment](https://github.com/xiaohw7/cgmoi/blob/main/Images/axis_alignment.JPG)

      * While running task Cg, mass and CG coordinates can be read straight off the output on the Serial monitor. Refer to "Calculations" section below for x and y axis. Coordinates are in mm.

      * User can send '14' to switch to mode 2 and measure MOI of satellite.

7. Mode 2: user intends on measuring MOI of satellite

      * Linear actuators must be fully retracted. Send '10' to fully retract linear actuators.

      * Ensure screw securing removeable shaft (shown above) is secured, send '14' to switch from mode 1 to mode 2.

      * Gyro and Motor tasks will run simultaneously and angular acceleration values can be read from serial monitor. Motor will spin back and forth to allow gyro to measure acceleration values. Once measurement process is finished, average acceleration will be displayed on serial monitor along with "Finish recording" message. User has 15 seconds to mount/dismount satellite before recording process starts again.

      * User should record down angular acceleration values with and without satellite mounted. Refer to "Calculations" section for the equations to obtain MOI.

      * While running mode 2, user can also send '15' to switch to mode 1 and return to reading values from load cells.

Below is an illustration of how to navigate mode 1 and mode 2

***
User uploads code

&darr; &darr; &darr;

Mode 1: Cg task begins, code outputs mass and CG values

&darr; &darr; &darr; *User send '14'*

Mode 2: Cg task stops, Motor and Gyro tasks begin, code outputs angular acceleration values

&darr; &darr; &darr; *User send '15'*

Motor and Gyro tasks stop, Cg task resumes, code outputs mass and CG values. Code reverts back to mode 1 above.
***

## Commands to send

|Mode |Commands that can be sent |What it does|
|-----|--------------------------|------------|
|1 |Send '1' |Turn on power to linear actuators|
|1 |Send '2' |Turn off power to linear actuators|
|1 |Send '3'/'5'/'7' |Move linear actuator 1/2/3 up for 1 second|
|1 |Send '4'/'6'/'8' |Move linear actuator 1/2/3 down for 1 second|
|1 |Send '9' |Move all 3 linear actuators up fully|
|1 |Send '10' |Move all 3 linear actuators down fully|
|1 |Send '11' |Tare all load cells|
|1 |Send '12' |Turn on power to stepper motor|
|1 |Send '13' |Turn off power to stepper motor|
|1 |Send '14' |End Cg task and resume/start Gyro and Motor tasks, switch to mode 2|
|2 |Send '12' |Turn on power to stepper motor|
|2 |Send '13' |Turn off power to stepper motor|
|2 |Send '15' |End Gyro and Motor tasks and resume Cg task, switch to mode 1|


## Points to take note

- Before raising/lowering linear actuators, user must remember to ***remove screw securing removeable shaft***.

- When lowering top plate, user should stand by and make sure removeable shaft enters designated slot correctly.

- Before measuring CG, user to ensure that the tips of adjustable screws (shown below) on load cells are the same distance above the table top so that when top plate is resting on load cells, top plate is as flat as possible.

![load cell screws](https://github.com/xiaohw7/cgmoi/blob/main/Images/load%20cell%20screws.JPG)

- Before measuring MOI, ensure screw securing removeable shaft is tightened to prevent any play when motor is turning.

- FreeRTOS can be buggy which causes starting position of gyroscope to shift after repeatedly switching from mode 1 to mode 2. Hence it is recommended to position gyro chip above red tape before uploading `cgmoi.ino`. User can rotate top plate a set number of steps using `Stepper_motor_driver2.ino`.

- There is a tendency for SPDT relay supplying power to stepper motor to get stuck in the close position despite the LED light being off and signal sent to it to disconnect. This may be because relay is only rated for 30V while power supply is at 36V. Tapping the blue box on the relay would help to disconnect it. User can tell if power had been disconnected by observing light on stepper motor driver.

- Angular acceleration values is rather unreliable and difference while loaded and unloaded may not be easily observed. Sometimes loaded values may even be higher than unloaded values. May be due to looseness and play at the epoxy region of removeable shaft.

- First measurements of angular velocity tend to be buggy and inaccurate, hence code only starts recording values from second reading onwards.

- If load cell readings become inaccurate, try running "Calibration" example from this [github library](https://github.com/olkal/HX711_ADC) and obtain calibration values for each individual load cell and input into the code.

- If there is a need to stop measurements/linear actuators/stepper motor immediately, unplug Arduino to "Emergency Stop".

## Calculations

![edited coordinates](https://github.com/xiaohw7/cgmoi/blob/main/Images/edited_coordinates_cgmoi.png)

With reference to image above, points A,B,C correspond to load cell 1,2,3 respectively. Load cell numbers are marked on load cells.

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
      * y1, y2, y3 are distances of points A, B, ad C from y-axis respectively. y1 = 206mm, y2 and y3 = 103mm
      * x1, x2, are distances of point B and C from x-axis. x1 and x2 = 178.4mm
      * Lengths are calculated using values from [load cell plate drawing](https://github.com/xiaohw7/cgmoi/blob/main/M02_load%20cell%20plate_drawing.pdf) and [load cell data sheet](https://github.com/xiaohw7/cgmoi/blob/main/M02_load%20cell%20plate_drawing.pdf).
      * To obtain z-coordinate of CG, rotate satellite by 90 degrees and undergo same calculations to obtain z-coordinate.
      * Code outputs coordinate of CG with reference to axis above and in millimeters.

3. Finding Moment Of Inertia

      * ![moi_eqn](https://github.com/xiaohw7/cgmoi/blob/main/Images/moi_eqn.png)

      * Use equation above to obtain J1. Where J1 is MOI of satellite. J0 is MOI of fixture tools which can be obtained from solid works. Shown in picture below, J0 in the vertical axis, taken at output coordinate system with center of top plate as origin, is ***19113908.45 grams * square millimeters***.
      * ![MOI_topplate](https://github.com/xiaohw7/cgmoi/blob/main/Images/MOI_topplate.JPG)
      *  Referring again to equation to obtain J1, w0 is angular velocity at time t without satellite. w’0 is angular velocity at time t with satellite. t1 and t2 are time at which angular velocity w0 and w’0 is taken respectively.
      * Thus it is observed that the fraction in the equation is angular acceleration without satellite mounted divided by angular acceleration with satellite mounted.
      * Code outputs angular acceleration values in degrees/sec^2 which can then be used to calculate J0.

      * Rotating about X-axis of satellite will result in JX

      * Rotating about Y-axis of satellite will result in JY

      * Rotating about Z-axis of satellite will result in JZ

      * ![parallel axis theorem](https://github.com/xiaohw7/cgmoi/blob/main/Images/parallel_axis_theorem.png)

      * Above equation uses parallel axis parallel axis theorem to correct MOI vector with reference to the CG and find vector of MOI. JxG, JyG, JzG are the MOI in x, y, and z direction respectively.

## Areas for improvement

1. Values for angular acceleration can vary substantially leading to inaccurate and imprecise values of MOI.

     * One way to improve is to move designated mounting place of BMI160 chip to the center of the top plate. This way Arduino can use raw values of angular velocity measured by the gyroscope in the MOI equation instead of having to measure linear acceleration values and then subject them to a series of calculations to obtain angular acceleration.

     * Another way is to decrease the play in the removeable shaft especially at the spot with Epoxy where the shaft connects with the flat plate.

2. Torque of stepper motor varies with speed and is difficult to control. This may lead to inaccurate values of MOI when torque applied while top plate is loaded vs when it is unloaded is different.

     *  Potential way to improve is to change to motor to a simple DC motor that outputs constant torque every time it accelerated top plate from rest regardless of whether top plate is loaded.

## Instructions
Below are instructions on how to set up each individual component of the cgmoi machine, links to each component's datasheet, as well as some notes I made on the problems I faced in the process.

### HX711 ADC and load cells:

- Refer to [this link](https://makersportal.com/blog/2019/5/12/arduino-weighing-scale-with-load-cell-and-hx711) for instructions on how to connect load cell to HX711 to Arduino. All 3 load cells can share a common power and ground connection.

- Download library at [github](https://github.com/olkal/HX711_ADC) for examples on calibrating and reading values from multiple load cells.

- To calibrate load cell, connect Arduino to an individual load cell through HX711 and run “Calibration” example. Follow instructions on the serial monitor and record down “calibration value”.  Do this for every individual load cell.

- Use `HX711_script3.ino` to read all 3 load cells. Input “calibration values” of each respective load cell into the code. Weight reading is taken 10 times with 0.05sec intervals and average weight on each load cell is obtained. Calculations are then made with the average weights. Run the code and read values of weight and coordinates of CG from serial monitor.

- Send “t” on serial monitor to tare load cells immediately.

- Reading is in grams. CG units are in mm.

- [HX711 data sheet](https://onedrive.live.com/?authkey=%21AlhODXygXZWnhUQ&cid=0008E16A592663A6&id=8E16A592663A6%2129427&parId=8E16A592663A6%2129388&o=OneUp)

- [load cell data sheet](https://onedrive.live.com/?authkey=%21AlhODXygXZWnhUQ&cid=0008E16A592663A6&id=8E16A592663A6%2129434&parId=8E16A592663A6%2129388&o=OneUp)

### BMI 160 Gyro:

- Refer to [hanyazou's github](https://github.com/hanyazou/BMI160-Arduino) for instructions to connect to gyro sensor. Download library at the same website. Use example `Gyro` example or `bmi_160_gyro_script.ino` to read from sensor. Switch example code to I2C mode.

- Another library that can read from gyro is [DFRobot](https://github.com/DFRobot/DFRobot_BMI160#installation). Download the library and use `accelgyro` example for both accelerometer and gyroscope readings.

- `bmi_160_gryo_script2.ino` takes `accelgyro` example from DFRobot library and modifies it to obtain net acceleration from x and y axis using Pythagoras theorem. Measurements are taken every 0.1 seconds and the average of 10 readings are calculated. Acceleration is converted from g to degrees/sec^2.

- `bmi_160_gryo_script3.ino` modifies `bmi_160_gyro_script.ino` to get net acceleration using Pythagoras theorem on the x an y axis acceleration values.

- Net acceleration is then divided by radius (distance from gyro to center of top plate) to obtain angular acceleration in radians which is then converted to degrees.

- Sensor is to use 3.3v from Arduino.

- Helpful links at [arduino projects](http://www.arduinoprojects.net/sensor-projects/using-bmi160-sensor-arduino-uno.php) and [sparkfun.](https://learn.sparkfun.com/tutorials/gyroscope/all)

- [BMI 160 datasheet](https://onedrive.live.com/?authkey=%21AlhODXygXZWnhUQ&cid=0008E16A592663A6&id=8E16A592663A6%2129414&parId=8E16A592663A6%2129388&o=OneUp)

### Linear actuator with relay:

- One SPDT relay to be used as overall power switch to linear actuators. 3 DPDT relays to be used as switches for each individual actuator and to be connected in a way to allow current to flow in both direction so as to enable actuator to extend and retract.

- Helpful links for SPDT relay can be found at [circuit basics](https://www.circuitbasics.com/setting-up-a-5v-relay-on-the-arduino/) and [last minute engineers](https://lastminuteengineers.com/one-channel-relay-module-arduino-tutorial/).

- Wiring for DPDT relay to be able to reverse current polarity to actuators can be found at [digikey](https://forum.digikey.com/t/polarity-reversal-using-a-dpdt-switch/626/4).

- Wiring:

![linear actuator wiring](https://github.com/xiaohw7/cgmoi/blob/main/Images/linear_actuator_wiring.png)

- Above picture shows wiring of linear actuators with relays.

- VCC and GND of all relays are connected to 5V and GND of Arduino through breadboard.

- For each DPDT relay, NO2 and NC1 should be connected to the black wire of actuator while NO1 and NC2 should be connected to the red wire of actuator.

- Use `linear_actuator_script.ino` to control the actuators by sending commands through serial monitor.

|Commands that can be sent |What it does|
|--------------------------|------------|
|Send '1'|Power on|
|Send '2'|Power off|
|Send '3'/'5'/'7'|Move actuator 1/2/3 up for 1 sec|
|Send '4'/'6'/'8'|Move actuator 1,2,3 down for 1 sec|
|Send '9'|Move all 3 actuators up completely|
|Send '10'|Move all 3 actuators down completely|

- Linear actuator to use 12V from power supply.

- [Linear actuator data sheet](https://onedrive.live.com/?authkey=%21AlhODXygXZWnhUQ&cid=0008E16A592663A6&id=8E16A592663A6%2129428&parId=8E16A592663A6%2129388&o=OneUp)

- [spdt relay datasheet](https://onedrive.live.com/?authkey=%21AlhODXygXZWnhUQ&cid=0008E16A592663A6&id=8E16A592663A6%2129439&parId=8E16A592663A6%2129388&o=OneUp)

- [dpdt relay datasheet](https://datasheetspdf.com/datasheet/HK19F-DC5V.html)

### LICHUAN LCDA257S stepper motor driver and stepper motor:

- See [dronebotworkship](https://dronebotworkshop.com/big-stepper-motors/) to get an overview on stepper drivers and motors.

- See [makersguides](https://www.makerguides.com/tb6600-stepper-motor-driver-arduino-tutorial/) for example code and wiring.

- Number of pulses per round can be adjusted by referring to table on the driver.

- Download AccelStepper library from [github](https://github.com/waspinator/AccelStepper).

- In AccelStepper library, speed is in steps/sec, Acceleration is in steps/sec2, position is in number of steps.

- `Stepper_motor_driver.ino` controls stepper motor directly while `Stepper_motor_driver2.ino` uses AccelStepper library to control motor.

- Another way of accelerating stepper motor is do gradually quicken pulses sent to motor (ramping). Code is in `Stepper_motor_driver3.ino`.

- Stepper motor to use 36V from power supply.

- [Stepper motor driver datasheet](https://onedrive.live.com/?authkey=%21AlhODXygXZWnhUQ&cid=0008E16A592663A6&id=8E16A592663A6%2129431&parId=8E16A592663A6%2129388&o=OneUp)

- [Stepper motor datasheet](https://onedrive.live.com/?authkey=%21AlhODXygXZWnhUQ&cid=0008E16A592663A6&id=8E16A592663A6%2129413&parId=8E16A592663A6%2129388&o=OneUp)

### Putting it together (Gyro and Motor):

- Function to control movement of stepper motor in AccelStepper library is blocking, thus we use FreeRTOS on Arduino, Gyro and Motor can run and read data simultaneously. FreeRtos library can be found at [github](https://github.com/feilipu/Arduino_FreeRTOS_Library).

- `moi.ino`(`Stepper_motor_driver2.ino` + `bmi_160_gryo_script3.ino`) uses FreeRTOS and AccelStepper library to run motor and gyro simultaneously.

- Alternatively, we can use Pulse Width Modulation to send pulses to stepper motor. analogWrite() function output a PWM signal to stepper motor and is not a blocking function. Hence can collect data from gyro immediately after starting stepper motor.

- Code to toggle PWM frequencies in Arduino can be found  [here](https://www.etechnophiles.com/how-to-change-pwm-frequency-of-arduino-mega/).

- Take note: code to change PWM frequency to 62500 Hz changes delay() function such that delay(1000000) now is approximately 15 sec.

- Also, offset values are recorded so as to account for gyro not being completely flat. Code can be found in `moi2.ino` (using PWM to control stepper motors + `bmi_160_gryo_script2.ino`).

- However, Gyro still experiences large amount of noise. To reduce noise, gyro code include a digital low pass filter in the code from [hanyazou's github](https://github.com/hanyazou/BMI160-Arduino).  Also, a simple Kalman Filter found on [github] (https://github.com/denyssene/SimpleKalmanFilter) is added into the code which decreased standard variation when top plate is stationary as well as when it is moving, the final angular acceleration value is put through the filter.  

- Using FreeRTOS and PWM with altered frequency of 31372.55 Hz, a graph of angular acceleration against time is plotted through the acceleration and deceleration of the stepper motor.  It is observed that there is a very sharp peak acceleration when stepper motor accelerates and acceleration. Hence the gyro will measure acceleration values throughout the motor’s acceleration and the code will record down the peak acceleration value.

- Gyroscope sample rate is changed from 200Hz to 1600Hz so as to increase accuracy. Peak acceleration values are put in an array and after a certain number or recordings, mean and standard deviation are calculated from the peak values in array. Code is in `moi3.ino` (using PWM to control stepper motors + `bmi_160_gryo_script3.ino`).

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

### Putting it together (Printed Circuit Board):

- The Arduino Mega board along with relays and ADCs are to be stacked on top of the PCB.

- Wiring of components of cgmoi machine such as load cells, gyro, ADC to Arduino on PCB will be done with screw terminals soldered onto PCB.

- Female headers will be soldered on to PCB so that relays and ADCs can be stacked on PCB.

- Male headers will be soldered on to PCB so that Arduino Mega can be stack on the PCB.

- PCB along with its components would be put into a PCB encloser with opening for wires to connect to other components.

- [Screw terminal to be used](https://www.digikey.sg/en/products/detail/w%C3%BCrth-elektronik/691211720002/2060530 )

- [Female headers to be used](https://www.digikey.sg/en/products/detail/samtec-inc/SSW-101-01-G-D/6691946 )

- [Male headers to be used](https://www.digikey.sg/en/products/detail/sullins-connector-solutions/PREC003SAAN-RC/2774851?s=N4IgTCBcDaIMoEYAMCwFEDCBaJBmLAcgCIgC6AvkA)

- [PCB encloser to be used](https://www.lazada.sg/products/extruded-pcb-aluminum-box-black-enclosure-electronic-project-case-80x160x170mm-i2083438345-s11541324037.html)

- There was an error in the PCB design regarding dpdt 2 and dpdt 3. The input, vcc, com1, and com2 pins meant for dpdt2 has been wrongly placed at dpdt3 and vice versa. Fortunately, all dpdt relays share the same vcc, com1, and com2 so technically there is no impact there. As for the input pins, simply changing the Arduino code such that input pin of dpdt 2 is now the input pin of dpdt 3 and vice versa has solved the problem.

- 12v power supply for linear actuators do not function properly when connected to PCB. When linear actuators are turned on, 12v power supply stutters and its LED light flicker on and off. As a result, linear actuators also stutter. Actuators are able to function normally when benchtop power supply is connected to PCB. Actuators are also able to function normally when 12v power supply is connected to breadboard. This occurrence seems to be due to the start-up current required by 3 linear actuators being too high for the 12v power supply when connected to PCB. The solution is to lower the output of the 12v power supply to roughly 10.5v. This decreased the start-up current that 3 linear actuators require to a manageable amount for the 12v power supply allowing it to continue supplying power to move the actuators.
