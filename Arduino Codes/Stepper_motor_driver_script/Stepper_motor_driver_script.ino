/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino without a library: continuous rotation. More info: https://www.makerguides.com */

// Define stepper motor connections:
#define dirPin 2
#define stepPin 4

// Time of half a pulse
int t = 1000;

void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // Set the spinning direction CW/CCW:
  digitalWrite(dirPin, LOW);
}

void loop() {
  // These four lines result in 1 step:
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(t);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(t);
}
