//load cell definition---------------------------------------------------------------------------------------
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#endif

//Kalman filter library ---------------------------------------------------------------------------
#include <SimpleKalmanFilter.h>

//pins:
const int HX711_dout_1 = 5; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 6; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 7; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 8; //mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 9; //mcu > HX711 no 3 dout pin
const int HX711_sck_3 = 10; //mcu > HX711 no 3 sck pin

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); //HX711 3
unsigned long t = 0;

//linear actuator definition--------------------------------------------------------------------------------------
int spdtSignal = 31; //power supply
int dpdtSignal1 = 32; //linear actuator 1
int dpdtSignal2 = 33; //linear actuator 2
int dpdtSignal3 = 34; //linear actuator 3

//Define kalman filters--------------------------------------------------------------------------
SimpleKalmanFilter massKalmanFilter(2, 2, 1);
SimpleKalmanFilter xCGKalmanFilter(0.7, 0.7, 0.9);
SimpleKalmanFilter yCGKalmanFilter(0.7, 0.7, 0.9);

void setup() {
//load cell set up--------------------------------------------------------------
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  float calibrationValue_1; // calibration value load cell 1
  float calibrationValue_2; // calibration value load cell 2
  float calibrationValue_3; // calibration value load cell 3
  
  calibrationValue_1 = 88.95; // uncomment this if you want to set this value in the sketch
  calibrationValue_2 = 89.10; // uncomment this if you want to set this value in the sketch
  calibrationValue_3 = 91.65; // uncomment this if you want to set this value in the sketch
  
  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();
  
  unsigned long stabilizingtime = 10000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  byte loadcell_3_rdy = 0;
  while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy) < 3) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  if (LoadCell_3.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  LoadCell_3.setCalFactor(calibrationValue_2); // user set calibration value (float)
  Serial.println("Startup is complete");

//linear actuator set up-----------------------------------------------------------------------------
  // Set pins as an output pin
  pinMode(spdtSignal, OUTPUT);
  pinMode(dpdtSignal1, OUTPUT);
  pinMode(dpdtSignal2, OUTPUT);
  pinMode(dpdtSignal3, OUTPUT);

  //Serial.begin(9600); //Serial already begun above
  //while(!Serial);
  digitalWrite(spdtSignal, HIGH); //turn off power
}

void loop() {

  //set totalA, totalB, totalC to 0
  float totalA = 0;
  float totalB = 0;
  float totalC = 0;

  for (int i = 0; i < 20; i++) {
    
    static boolean newDataReady = 0;
    //const int serialPrintInterval = 1000; //increase value to slow down serial print activity

     // check for new data/start next conversion:
     //if (LoadCell_1.update()) newDataReady = true;
     //LoadCell_2.update();
     //LoadCell_3.update();

     LoadCell_1.update();
     LoadCell_2.update();
     LoadCell_3.update();
     newDataReady = true;
  
     //get smoothed value from data set
     if ((newDataReady)) {
       //if (millis() > t + serialPrintInterval) {
         float a = LoadCell_1.getData();
         float b = LoadCell_2.getData();
         float c = LoadCell_3.getData();
         //Serial.print("Load_cell 1 output val: ");
         //Serial.println(a);
         //Serial.print("    Load_cell 2 output val: ");
         //Serial.println(b);
         //Serial.print("    Load_cell 3 output val: ");
         //Serial.println(c);
         totalA = totalA + a;
         totalB = totalB + b;
         totalC = totalC + c;
         delay(50);
         newDataReady = 0;
     }
  }
        
     

       float averageA = totalA/20;
       float averageB = totalB/20;
       float averageC = totalC/20;
       float M  = (averageA + averageB + averageC); // mass of satellite
       //float M = a + b + c; // mass of satellite

       //Run mass through kalman filter
       M = massKalmanFilter.updateEstimate(M);
       Serial.print("Mass of satellite(g): ");
       Serial.println(M);
       float yCoordinate = (averageB*103 + averageC*103 - averageA*206)/M; // y-coordinate of CG
       float xCoordinate = (averageB*178.4 - averageC*178.4)/M; // x-coordinate of CG
      
       //Run x and y coordinate through kalman filter
       xCoordinate = xCGKalmanFilter.updateEstimate(xCoordinate);
       yCoordinate = yCGKalmanFilter.updateEstimate(yCoordinate);
      
       Serial.print("Coordinates of CG(mm): (");
       Serial.print(xCoordinate);
       Serial.print(",");
       Serial.print(yCoordinate);
       Serial.println(")");
         //newDataReady = 0;
         //t = millis();
         
       //} 
      //}
    //} 
     

    

  delay(1000);

  // receive command from serial terminal
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
      delay(1000);   
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off"); 
    }

    //send "4" to move actuator 1 down for 1 sec
    else if (incomingByte == 4) { 
      digitalWrite(dpdtSignal1, LOW); //set actuator 1 to downward position
      delay(500);   
      digitalWrite(spdtSignal, LOW); //turn on spdt relay
      Serial.println("Power on. Move actuator 1 down.");
      delay(1000);
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
      delay(50000);
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
      delay(50000);
      digitalWrite(spdtSignal, HIGH); //turn off spdt relay
      Serial.println("Power off");
    }
    
    //send "11" to initiate tare operation on load cells
    else if (incomingByte == 11) {
      Serial.println("Tare load cells"); 
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
      LoadCell_3.tareNoDelay();
    }
  }
    //check if last tare operation is complete
    if (LoadCell_1.getTareStatus() == true) {
      Serial.println("Tare load cell 1 complete");
    }
    if (LoadCell_2.getTareStatus() == true) {
      Serial.println("Tare load cell 2 complete");
    }
    if (LoadCell_3.getTareStatus() == true) {
      Serial.println("Tare load cell 3 complete");
    }     
  }



  
 
