/* Example sketch to control a stepper motor with TB6600 stepper motor driver, AccelStepper library and Arduino: acceleration and deceleration. More info: https://www.makerguides.com */

// Include the AccelStepper library:
#include <AccelStepper.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2
#define stepPin 4
#define motorInterfaceType 1
#define stepperMotorSignal 22

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(16000);
  stepper.setAcceleration(100);
  pinMode(stepperMotorSignal, OUTPUT);
  //Turn on power to stepper motor (LOW = on, HIGH = off)
  digitalWrite(stepperMotorSignal, LOW);
  Serial.begin(9600);
}

void loop() {
  // Set the target position(positive = cw, negative = ccw)
  stepper.moveTo(4000);
  //Run to target position with set speed and acceleration/deceleration:
  stepper.runToPosition();

  //delayMicroseconds(10);

  // Move back to zero:
  //stepper.moveTo(0);
  //stepper.runToPosition();

  //delayMicroseconds(10);

  delay(3000);

  //receive signal from serial monitor
  if (Serial.available() > 0) {
    int incomingByte = Serial.parseInt();
        
    if (incomingByte == 12) { //send "12" to turn on power to stepper motor
     digitalWrite(stepperMotorSignal, LOW);
     Serial.print("Stepper Motor power on"); 
    }

    else if (incomingByte == 13) { //send "13" to turn off power to stepper motor
      digitalWrite(stepperMotorSignal, HIGH);
      Serial.print("Stepper Motor power off");
    }
  }
  
}
