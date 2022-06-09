int spdtSignal = 31; //power supply
int dpdtSignal1 = 32; //linear actuator 1
int dpdtSignal2 = 34; //linear actuator 2
int dpdtSignal3 = 33; //linear actuator 3

void setup() {
  // Set pins as an output pin
  pinMode(spdtSignal, OUTPUT);
  pinMode(dpdtSignal1, OUTPUT);
  pinMode(dpdtSignal2, OUTPUT);
  pinMode(dpdtSignal3, OUTPUT);

  Serial.begin(9600);
  while(!Serial);
  digitalWrite(spdtSignal, HIGH); //turn off power
  
}

void loop() {
  if (Serial.available() > 0) {
    int incomingByte = Serial.parseInt();

    if (incomingByte == 1) { //send "1" to turn on spdt relay
      digitalWrite(spdtSignal, LOW);
      Serial.println("Power on");      
    }
    else if (incomingByte == 2) { //send "2" to turn off spdt relay
      digitalWrite(spdtSignal, HIGH);
      Serial.println("Power off");
    }

    //send "3" to move actuator 1 up for 1 sec
    else if (incomingByte == 3) { 
      digitalWrite(dpdtSignal1, HIGH); //set actuator 1 to upward position
      delay(500); //delay time for relay to switch position
      digitalWrite(spdtSignal, LOW); //turn on spdt relay
      Serial.println("Power on. Move actuator 1 up.");
      delay(3000);   
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off"); 
    }

    //send "4" to move actuator 1 down for 1 sec
    else if (incomingByte == 4) { 
      digitalWrite(dpdtSignal1, LOW); //set actuator 1 to downward position
      delay(500);   
      digitalWrite(spdtSignal, LOW); //turn on spdt relay
      Serial.println("Power on. Move actuator 1 down.");
      delay(3000);
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off");
    }

    //send "5" to move actuator 2 up for 1 sec
    else if (incomingByte == 5) { 
      digitalWrite(dpdtSignal2, HIGH); //set actuator 2 to upward position
      delay(500); 
      digitalWrite(spdtSignal, LOW); //turn on spdt relay
      Serial.println("Power on. Move actuator 2 up.");
      delay(1000);  
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off");
    }

    //send "6" to move actuator 2 down for 1 sec
    else if (incomingByte == 6) { 
      digitalWrite(dpdtSignal2, LOW); //set actuator 2 to downward position
      delay(500); 
      digitalWrite(spdtSignal, LOW); //turn on spdt relay
      Serial.println("Power on. Move actuator 2 down.");
      delay(1000);  
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off");
    }

    //send "7" to move actuator 3 up for 1 sec
    else if (incomingByte == 7) { 
      digitalWrite(dpdtSignal3, HIGH); //set actuator 3 to upward position
      delay(500);  
      digitalWrite(spdtSignal, LOW); //turn on spdt relay
      Serial.println("Power on. Move actuator 3 up.");
      delay(1000); 
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off");
    }

    //send "8" to move actuator 3 down for 1 sec
    else if (incomingByte == 8) { 
      digitalWrite(dpdtSignal3, LOW); //set actuator 3 to downward position
      delay(500); 
      digitalWrite(spdtSignal, LOW); //turn on spdt relay
      Serial.println("Power on. Move actuator 3 down");
      delay(1000);  
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off");
    }

    //send "9" to move platform up completely
    else if (incomingByte == 9) { //all actuators up
      digitalWrite(dpdtSignal3, HIGH);
      digitalWrite(dpdtSignal2, HIGH);
      digitalWrite(dpdtSignal1, HIGH);
      digitalWrite(spdtSignal, LOW); //turn on spdt relay
      delay(2000);
      Serial.println("Power on");
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off");
    }

    //send "10" to move platform down completely
    else if (incomingByte == 10) { //all actuators down
      digitalWrite(dpdtSignal3, LOW);
      digitalWrite(dpdtSignal2, LOW);
      digitalWrite(dpdtSignal1, LOW);
      digitalWrite(spdtSignal, LOW); //turn on spdt relay
      Serial.println("Power on");
      delay(3000);
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off");
    }
  }
}
  
