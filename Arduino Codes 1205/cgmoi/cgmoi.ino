#include <Arduino_FreeRTOS.h> //FreeRTOS library
#include <BMI160Gen.h> //Gyro library
#include <SimpleKalmanFilter.h> //Kalman filter library
#include <AccelStepper.h> //AccelStepper library
#include <HX711_ADC.h> //Load cell library
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#endif

//FreeRTOS definitions---------------------------------------------------------------------------
void TaskGyro( void *pvParameters );
void TaskMotor( void *pvParameters );
void TaskCg(void *pvParameters);

//define task handle
TaskHandle_t Gyro;
TaskHandle_t Motor;
TaskHandle_t Cg;

//Gyro definitions--------------------------------------------------------------------------------
const int select_pin = 10;
const int i2c_addr = 0x69;
uint8_t OSR4 = OSR4;

int loopCount = 0; //counting number of times Gyro task loops to get number of readings obtained by readAccelerometer function
float totalAccelDeg = 0;
float avgAccelDeg = 0;
float peakAccel = 0;

int xAccelRawOffset, yAccelRawOffset, zAccelRawOffset; //Raw offset data
int xAccelRaw, yAccelRaw, zAccelRaw; //Raw actual data
float totalYAccelOffset = 0; 
float totalXAccelOffset = 0;

//Step motor definitions----------------------------------------------------------------------------------------------------------------------
// Define stepper motor connections:
#define dirPin 2
#define stepPin 4
#define motorInterfaceType 1
#define stepperMotorSignal 22

// Create a new instance of the AccelStepper class---------------------------------------------------------------------------------------------
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

//Load cell definitions----------------------------------------------------------------------------------------------------
//pins:
const int HX711_dout_1 = 45; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 44; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 43; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 42; //mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 41; //mcu > HX711 no 3 dout pin
const int HX711_sck_3 = 40; //mcu > HX711 no 3 sck pin

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

//Define kalman filters---------------------------------------------------------------------------------------------
SimpleKalmanFilter massKalmanFilter(2, 2, 1);
SimpleKalmanFilter xCGKalmanFilter(0.7, 0.7, 0.9);
SimpleKalmanFilter yCGKalmanFilter(0.7, 0.7, 0.9);
SimpleKalmanFilter angularAccelDegFilter(1, 1, 0.5);

void setup() {
  
  Serial.begin(9600);
  while (!Serial);
  
//Set up tasks to run independently----------------------------------------------------------------------------------------
   xTaskCreate(
    TaskCg
    ,  "Cg"
    ,  128
    ,  NULL
    ,  2
    ,  &Cg);  
   
   xTaskCreate(
    TaskMotor
    ,  "Motor"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &Motor);
  
  xTaskCreate(
   TaskGyro
   ,  "Gyro"   // A name just for humans
   ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
   ,  NULL
   ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
   ,  &Gyro );
   
//Set up Gyro----------------------------------------------------------------------------------------------------
  // initialize device
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  // Set the accelerometer range to 4g
  BMI160.setAccelerometerRange(4);
  //Serial.println("Initializing IMU device...done.");

  //Set digital low pass filter for gyro
  BMI160.setAccelDLPFMode(OSR4);

  //Set sampling rate to 1600Hz
  BMI160.setAccelRate(12);
  
  //Calibrating accelerometer offset
  //Serial.println("Calibrating accelerometer");
  BMI160.autoCalibrateXAccelOffset(0);
  BMI160.setAccelOffsetEnabled(true);
  BMI160.autoCalibrateYAccelOffset(0);
  BMI160.setAccelOffsetEnabled(true);
  //BMI160.autoCalibrateZAccelOffset(-1);
  //BMI160.setAccelOffsetEnabled(true);

  //Measure offset values before motor starts
  for (int i = 0; i < 20; i++) { //collect data from accelerometer to calculate offset values
    BMI160.readAccelerometer(xAccelRawOffset, yAccelRawOffset, zAccelRawOffset);
    
    //convert raw data to g
    float xAccelOffset = convertRawGyro(xAccelRawOffset);
    float yAccelOffset = convertRawGyro(yAccelRawOffset);
    float zAccelOffset = convertRawGyro(zAccelRawOffset);
    totalXAccelOffset = totalXAccelOffset + xAccelOffset;
    totalYAccelOffset = totalYAccelOffset + yAccelOffset;
    delay(100);
  }

  float avgXOffset = totalXAccelOffset/20;
  float avgYOffset = totalYAccelOffset/20;
  float netOffset = sqrt(sq(avgXOffset) + sq(avgYOffset));
  Serial.println("Gyro start up is complete");

//Set up motor-------------------------------------------------------------------------------------------------------------------
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepperMotorSignal, OUTPUT);

  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(99999999);
  stepper.setAcceleration(999999);

  //Turn off power to stepper motor
  digitalWrite(stepperMotorSignal, HIGH);
  Serial.println("Motor start up is complete");

//Set up load cell------------------------------------------------------------------------------------------------------
  float calibrationValue_1; // calibration value load cell 1
  float calibrationValue_2; // calibration value load cell 2
  float calibrationValue_3; // calibration value load cell 3
  
  calibrationValue_1 = 88.80; // uncomment this if you want to set this value in the sketch
  calibrationValue_2 = 90.59; // uncomment this if you want to set this value in the sketch
  calibrationValue_3 = 91.09; // uncomment this if you want to set this value in the sketch
  
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
  Serial.println("Load cell Startup is complete");

//Linear actuator set up---------------------------------------------------------------------------------------------------------------
  // Set pins as an output pin
  pinMode(spdtSignal, OUTPUT);
  pinMode(dpdtSignal1, OUTPUT);
  pinMode(dpdtSignal2, OUTPUT);
  pinMode(dpdtSignal3, OUTPUT);

  digitalWrite(spdtSignal, HIGH); //turn off power to linear actuators
  Serial.println("Linear actuator start up is complete");

  Serial.println("Set up finish");

  //first suspend both Gyro and Motor tasks and turn off power to motor   
  vTaskSuspend(Gyro);    
  digitalWrite(stepperMotorSignal, HIGH);
  vTaskSuspend(Motor);
  
  Serial.println("Suspend motor and gyro, ready for input");
  
}

void loop() {
//nothing in loop
}

//Gyro task-----------------------------------------------------------------------------------------------------------------
void TaskGyro(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    if (loopCount < 30) { //By limitting loopCount, gyro only record during acceleration process and neglecting the deceleration process
      // add 1 to loop count everytime loop happens
      loopCount = loopCount + 1;
      //Serial.print("loop: ");
      //Serial.println(loopCount);
    
      BMI160.readAccelerometer(xAccelRaw, yAccelRaw, zAccelRaw);

      //convert raw data to g
      float xAccel = convertRawGyro(xAccelRaw);
      float yAccel = convertRawGyro(yAccelRaw);

      float netAccel = sqrt(sq(xAccel) + sq(yAccel)); // net acceleration
      netAccel = netAccel; //Minus off offset value
      float angularAccelRad = (netAccel * 9.807)/0.185; //angular acceleration is acceleration in g converted to m/s^2 divided by radius(dist from chip to center of top plate)
      //Serial.println(angularAccelRad);
      float angularAccelDeg = (angularAccelRad/3.14159265359)*180;

      //Run value through Kalman filter
      angularAccelDeg = angularAccelDegFilter.updateEstimate(angularAccelDeg);
  
      //Serial.println(angularAccelDeg);
 
      //add angularAccelDeg to total tally 
      totalAccelDeg = totalAccelDeg + angularAccelDeg;
      //avgAccelDeg = totalAccelDeg / loopCount;
      //Serial.println(avgAccelDeg);
  
      /*//get peak acceleration
      if (angularAccelDeg > peakAccel) {
        peakAccel = angularAccelDeg;
      }*/
      
      vTaskDelay(1);
    }

    else {
      vTaskDelay(1);
    }
  }
}

//Motor task----------------------------------------------------------------------------------------------------------
void TaskMotor(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    float rslts[10]; //array of 10 peak values from 10 measurements
    //Serial.println("Recording in progress...");

    //Record 10 values and add to rslts array. Since first recording tends to be inaccurate, i is set to 11 to so that the first reading is not added to array
    for (int i = 0; i < 11; i++) {
   
      // Set the target position:
      stepper.moveTo(4000);
      //Serial.println("set move to position");
    
      //resume Gyro task
      Serial.println("resume gyro");
      vTaskResume(Gyro);
           
      //Run to target position with set speed and acceleration/deceleration:
      stepper.runToPosition();

      //suspend Gyro task
      vTaskSuspend(Gyro);
      Serial.println("suspend gyro");
      
      //get average acceleration
      avgAccelDeg = totalAccelDeg / 30;

      //print average accel
      Serial.print("average acceleration: ");
      Serial.println(avgAccelDeg);

      //add only the second value onwards to array
      if (i > 0) {
        rslts[i-1] = avgAccelDeg;
        }
      
      vTaskDelay(1000/portTICK_PERIOD_MS);

      // Move back to zero:
      stepper.moveTo(0);
      stepper.runToPosition();
      //Serial.println("back to origin");

      vTaskDelay(1000/portTICK_PERIOD_MS);

      //reset values to 0
      loopCount = 0;
      peakAccel = 0;
      avgAccelDeg = 0;
      totalAccelDeg = 0;
      //Serial.print("reset");   
    }

    //Calculate mean value from array
    
    float totaccelarray = 0;
    float meanaccelarray = 0;
    for (int y = 0; y < 10; y++) {
      totaccelarray = totaccelarray + rslts[y];
      //Serial.println(rslts[y]);
    }
    meanaccelarray = totaccelarray/10;
    Serial.print("Mean: ");
    Serial.println(meanaccelarray);

    Serial.println("Finish recording");
    Serial.println("10 seconds until recording resumes");
    vTaskDelay(10000/portTICK_PERIOD_MS);


    //receive command from serial monitor
    if (Serial.available() > 0) {
      int incomingByte = Serial.parseInt();
        
      if (incomingByte == 12) { //send "12" to turn on power to stepper motor
        digitalWrite(stepperMotorSignal, LOW);
        Serial.println("Stepper Motor power on"); 
       }

      else if (incomingByte == 13) { //send "13" to turn off power to stepper motor
         digitalWrite(stepperMotorSignal, HIGH);
         Serial.println("Stepper Motor power off");
      }

      else if (incomingByte == 15) { //send "15" to suspend Gyro and Motor tasks and resume control task
        Serial.println("Suspend gyro and motor, resume Cg, ready for input");
        vTaskSuspend(Gyro);
        digitalWrite(stepperMotorSignal, HIGH);
        vTaskResume(Cg);
        vTaskSuspend(Motor);
      }
    }
  }
}

//Cg task------------------------------------------------------------------------------------------------------------------------
void TaskCg(void *pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    //A, B, C correspond to load cell 1, 2, 3 respectively
    //set totalA, totalB, totalC to 0
    float totalA = 0;
    float totalB = 0;
    float totalC = 0;

    for (int i = 0; i < 20; i++) {
      static boolean newDataReady = 0;
      //const int serialPrintInterval = 1000; //increase value to slow down serial print activity

       // check for new data/start next conversion:    
       LoadCell_1.update();
       LoadCell_2.update();
       LoadCell_3.update();
       newDataReady = true;
  
       //get smoothed value from data set
       if ((newDataReady)) {
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
    //Get average reading 
    float averageA = totalA/20;
    float averageB = totalB/20;
    float averageC = totalC/20;
    float M  = (averageA + averageB + averageC); // mass of satellite
       

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

    vTaskDelay(1000/portTICK_PERIOD_MS);

    //Receive command from serial terminal
    if (Serial.available() > 0) {
      int incomingByte = Serial.parseInt();

      if (incomingByte == 1) { //send "1" to turn on spdt relay
      digitalWrite(spdtSignal, LOW);
      Serial.println("Linear actuator power on");      
      }
      else if (incomingByte == 2) { //send "2" to turn off spdt relay
        digitalWrite(spdtSignal, HIGH);
        Serial.println("Linear actuator power off");
      }

      //send "3" to move actuator 1 up for 1 sec
      else if (incomingByte == 3) { 
        digitalWrite(dpdtSignal1, HIGH); //set actuator 1 to upward position
        delay(500); //delay time for relay to switch position
        digitalWrite(spdtSignal, LOW); //turn on spdt relay
        Serial.println("Linear actuator power on. Move actuator 1 up for 1 sec.");
        delay(1000);   
        digitalWrite(spdtSignal, HIGH); //turn off spdt relay
        Serial.println("Linear actuator power off"); 
      }

      //send "4" to move actuator 1 down for 1 sec
      else if (incomingByte == 4) { 
        digitalWrite(dpdtSignal1, LOW); //set actuator 1 to downward position
        delay(500);   
        digitalWrite(spdtSignal, LOW); //turn on spdt relay
        Serial.println("Linear actuator power on. Move actuator 1 down for 1 sec.");
        delay(1000);
        digitalWrite(spdtSignal, HIGH); //turn off spdt relay
        Serial.println("Linear actuator power off");
      }

      //send "5" to move actuator 2 up for 1 sec
      else if (incomingByte == 5) { 
        digitalWrite(dpdtSignal2, HIGH); //set actuator 2 to upward position
        delay(500); 
        digitalWrite(spdtSignal, LOW); //turn on spdt relay
        Serial.println("Linear actuator power on. Move actuator 2 up for 1 sec.");
        delay(1000);  
        digitalWrite(spdtSignal, HIGH); //turn off spdt relay
        Serial.println("Linear actuator power off");
      }
 
      //send "6" to move actuator 2 down for 1 sec
      else if (incomingByte == 6) { 
        digitalWrite(dpdtSignal2, LOW); //set actuator 2 to downward position
        delay(500); 
        digitalWrite(spdtSignal, LOW); //turn on spdt relay
        Serial.println("Linear actuator power on. Move actuator 2 down for 1 sec.");
        delay(1000);  
        digitalWrite(spdtSignal, HIGH); //turn off spdt relay
        Serial.println("Linear actuator power off");
      }

      //send "7" to move actuator 3 up for 1 sec
      else if (incomingByte == 7) { 
        digitalWrite(dpdtSignal3, HIGH); //set actuator 3 to upward position
        delay(500);  
        digitalWrite(spdtSignal, LOW); //turn on spdt relay
        Serial.println("Linear actuator power on. Move actuator 3 up for 1 sec.");
        delay(1000); 
        digitalWrite(spdtSignal, HIGH); //turn off spdt relay
        Serial.println("Linear actuator power off");
      }

      //send "8" to move actuator 3 down for 1 sec
      else if (incomingByte == 8) { 
        digitalWrite(dpdtSignal3, LOW); //set actuator 3 to downward position
        delay(500); 
        digitalWrite(spdtSignal, LOW); //turn on spdt relay
        Serial.println("Linear actuator power on. Move actuator 3 down for 1 sec.");
        delay(1000);  
        digitalWrite(spdtSignal, HIGH); //turn off spdt relay
        Serial.println("Linear actuator power off");
      }

      //send "9" to move platform up completely
      else if (incomingByte == 9) { //all actuators up
        digitalWrite(dpdtSignal3, HIGH);
        digitalWrite(dpdtSignal2, HIGH);
        digitalWrite(dpdtSignal1, HIGH);
        digitalWrite(spdtSignal, LOW); //turn on spdt relay
        Serial.println("Linear actuator power on. Raise platform completely.");
        Serial.println("Make sure screw securing removeable shaft has been removed!");
        delay(60000);        
        digitalWrite(spdtSignal, HIGH); //turn off spdt relay
        Serial.println("Linear actuator power off");
      }
  
      //send "10" to move platform down completely
      else if (incomingByte == 10) { //all actuators down
        digitalWrite(dpdtSignal3, LOW);
        digitalWrite(dpdtSignal2, LOW);
        digitalWrite(dpdtSignal1, LOW);
        digitalWrite(spdtSignal, LOW); //turn on spdt relay
        Serial.println("Linear actuator power on. Lower platform completely.");
        Serial.println("Make sure screw securing removeable shaft has been removed!");
        delay(60000);
        digitalWrite(spdtSignal, HIGH); //turn off spdt relay
        Serial.println("Linear actuator power off");
      }
      
      //send "11" to initiate tare operation on load cells
      else if (incomingByte == 11) {
        Serial.println("Tare load cells"); 
        LoadCell_1.tareNoDelay();
        LoadCell_2.tareNoDelay();
        LoadCell_3.tareNoDelay();
      }

      //send "12" to turn on power to stepper motor
      if (incomingByte == 12) { 
        digitalWrite(stepperMotorSignal, LOW);
        Serial.println("Stepper Motor power on"); 
       }

      //send "13" to turn off power to stepper motor
      else if (incomingByte == 13) { //send "13" to turn off power to stepper motor
         digitalWrite(stepperMotorSignal, HIGH);
         Serial.println("Stepper Motor power off");
      }

      //send "14" to resume Gyro and Motor tasks and suspend Control task and turn on power to motor
      if (incomingByte == 14) { 
        Serial.println("Resume motor and gyro");
        digitalWrite(stepperMotorSignal, LOW); //turn on power to motor

        //reset values to 0
        loopCount = 0;
        //totalAccelDeg = 0;
        //avgAccelDeg = 0;
        //Serial.print("reset");
        
        vTaskResume(Motor);
        //TaskResume(Gyro);     
        vTaskSuspend(Cg);
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
}




//Function to convert raw data from gyro------------------------------------------------------------
float convertRawGyro(int gRaw) {
  // since we are using 4 g range
  // -4 maps to a raw value of -32768
  // +4 maps to a raw value of 32767

  float g = (gRaw * 4) / 32768.0;

  return g;
}
