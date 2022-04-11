#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//pins:
const int HX711_dout_1 = 45; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 44; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 7; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 8; //mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 9; //mcu > HX711 no 3 dout pin
const int HX711_sck_3 = 10; //mcu > HX711 no 3 sck pin

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); //HX711 3

//const int calVal_eepromAdress_1 = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
//const int calVal_eepromAdress_2 = 4; // eeprom adress for calibration value load cell 2 (4 bytes)
unsigned long t = 0;

void setup() {
  Serial.begin(57600); delay(10);
  //Serial.println();
  //Serial.println("Starting...");

  float calibrationValue_1; // calibration value load cell 1
  float calibrationValue_2; // calibration value load cell 2
  float calibrationValue_3; // calibration value load cell 3
  
  calibrationValue_1 = 89.6; // uncomment this if you want to set this value in the sketch
  calibrationValue_2 = 91.12; // uncomment this if you want to set this value in the sketch
  calibrationValue_3 = 90.18; // uncomment this if you want to set this value in the sketch

  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();
  
  unsigned long stabilizingtime = 5000; // tare preciscion can be improved by adding a few seconds of stabilizing time
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
  //Serial.println("Startup is complete");
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 50; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();
  LoadCell_3.update();

  //set totalA, totalB, totalC to 0
  float totalA = 0;
  float totalB = 0;
  float totalC = 0;
  
  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t + serialPrintInterval) {
      //for (int i = 0; i < 10; i++) { // take 10 readings and obtain average
        float a = LoadCell_1.getData();
        float b = LoadCell_2.getData();
        float c = LoadCell_3.getData();
        //Serial.print("Load_cell 1 output val: ");
        Serial.print(a);
        Serial.print("    Load_cell 2 output val: ");
        Serial.print(b);
        Serial.print("    Load_cell 3 output val: ");
        Serial.println(c);
        //totalA = totalA + a;
        //totalB = totalB + b;
        //totalC = totalC + c;
        //delay(50);
      //}
      //float averageA = totalA/10;
      //float averageB = totalB/10;
      //float averageC = totalC/10;
      //float M  = (averageA + averageB + averageC); // mass of satellite
      float M = a + b + c; //mass of satellite
      //Serial.print("Mass of satellite(g): ");
      //Serial.println(M);
      //float yCoordinate = (averageB*103 + averageC*103 - averageA*206)/M; // y-coordinate of CG
      //float xCoordinate = (averageB*178.4 - averageC*178.4)/M; // x-coordinate of CG
      //Serial.print("Coordinates of CG(mm): (");
      //Serial.print(xCoordinate);
      //Serial.print(",");
      //Serial.print(yCoordinate);
      //Serial.println(")");
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
      LoadCell_3.tareNoDelay();
    }
  }

  //check if last tare operation is complete
  if (LoadCell_1.getTareStatus() == true) {
    //Serial.println("Tare load cell 1 complete");
  }
  if (LoadCell_2.getTareStatus() == true) {
    //Serial.println("Tare load cell 2 complete");
  }
  if (LoadCell_3.getTareStatus() == true) {
    //Serial.println("Tare load cell 3 complete");
  }

}
