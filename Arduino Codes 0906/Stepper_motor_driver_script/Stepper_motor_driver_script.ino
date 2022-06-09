/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino without a library: continuous rotation. More info: https://www.makerguides.com */

// Define stepper motor connections:
#define dirPin 2
#define stepPin 4
#define stepperMotorSignal 22

// Time of half a pulse
int t = 1000;

void setup() {
  Serial.begin(9600);
  
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepperMotorSignal, OUTPUT);
  
  // Set the spinning direction CW/CCW (LOW = CCW, HIGH = CW)
  digitalWrite(dirPin, LOW);

  //Turn off power to stepper motor
  digitalWrite(stepperMotorSignal, HIGH);
}

void loop() {
  // These four lines result in 1 step:
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(t);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(t);

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
