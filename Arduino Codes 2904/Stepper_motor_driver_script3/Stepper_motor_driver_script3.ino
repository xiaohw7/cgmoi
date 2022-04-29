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
  
  // Set the spinning direction CW/CCW:
  digitalWrite(dirPin, HIGH);

  //Turn off power to stepper motor
  digitalWrite(stepperMotorSignal, LOW);
}


void loop() {
  //acceleration of motor
  for (int i = 0; i < 3000; i++) {
    if (i < 1250||i > 1750) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(abs(1500 - i));
      digitalWrite(stepPin, LOW);
      delayMicroseconds(abs(1500 - i)); 
    }
    else {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(250);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(250); 
    }
       
  }
  //decceleration of motor
  //for (int x = 0; x < 1500; x++) {
    //digitalWrite(stepPin, HIGH);
    //delayMicroseconds(50 + (x/2));
    //digitalWrite(stepPin, LOW);
    //delayMicroseconds(50 + (x/2));
  //}
  delay(5000);

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
  delay(500);

}
