#include <BMI160Gen.h> //Gyro library
#include <SimpleKalmanFilter.h> //Kalman filter library

//Gyro definitions--------------------------------------------------------------------------------
const int select_pin = 10;
const int i2c_addr = 0x69;
uint8_t OSR4 = OSR4;

//define Kalman filter
SimpleKalmanFilter simpleKalmanFilter(13, 13, 0.7);

//Step motor definitions-----------------------------------------------------------------------------
// Define stepper motor connections:
#define dirPin 2
#define stepPin 4

void setup() {
//Set up Gyro----------------------------------------------------------------------------------------------------
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  // Set the accelerometer range to 4g
  BMI160.setAccelerometerRange(4);
  Serial.println("Initializing IMU device...done.");

  //Set filter for gyro
  BMI160.setAccelDLPFMode(OSR4);

  //Calibrating accelerometer offset
  Serial.println("Calibrating accelerometer");
  BMI160.autoCalibrateXAccelOffset(0);
  //BMI160.setAccelOffsetEnabled(true);
  BMI160.autoCalibrateYAccelOffset(0);
  //BMI160.setAccelOffsetEnabled(true);
  //BMI160.autoCalibrateZAccelOffset(-1);
  BMI160.setAccelOffsetEnabled(true);
  delay(1000);
  
  delay(500);
  Serial.println("Accelerometer offset enabled");

//Set up motor------------------------------------------------------------------------------
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  //change pwm frequency
  TCCR5B = TCCR5B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
  
  // Set the spinning direction to CCW:
  //digitalWrite(dirPin, LOW);
}

void loop() {
  //Set the spinning direction to CW
  digitalWrite(dirPin, HIGH);
  
  //Variables for Gyro readings
  int xAccelRawOffset, yAccelRawOffset, zAccelRawOffset; //Raw offset data
  int xAccelRaw, yAccelRaw, zAccelRaw; //Raw actual data
  float totalYAccelOffset = 0; 
  float totalXAccelOffset = 0;
  float totalYAccel = 0;
  float totalXAccel = 0;
  float rslts[10]; //array of 10 peak values from 10 measurements

  
  //Measure offset values before motor starts
  for (int i = 0; i < 20; i++) { //collect data from accelerometer to calculate offset values
    BMI160.readAccelerometer(xAccelRawOffset, yAccelRawOffset, zAccelRawOffset);
    
    //convert raw data to g
    float xAccelOffset = convertRawGyro(xAccelRawOffset);
    float yAccelOffset = convertRawGyro(yAccelRawOffset);
    float zAccelOffset = convertRawGyro(zAccelRawOffset);
    //Serial.println("OFFSET VALUES");
    //Serial.print(xAccelOffset);Serial.print("\t");
    //Serial.print(yAccelOffset);Serial.print("\t");
    //Serial.print(zAccelOffset);Serial.print("\t");
    //Serial.println();
    totalXAccelOffset = totalXAccelOffset + xAccelOffset;
    totalYAccelOffset = totalYAccelOffset + yAccelOffset;
    //Serial.print("total accel offset(x,y): ");
    //Serial.print(totalXAccelOffset);Serial.print("\t");
    //Serial.println(totalYAccelOffset);
    delay(10);
  }
  
  delay(10);

  for (int x = 0; x < 10; x++) {//Run 10 times to collect 10 peak values for rslts array
    float peakValue = 0; //peak value of one measurement
    
    digitalWrite(dirPin, HIGH); //Set spinning direction to CCW   
    analogWrite(stepPin,127); //Run stepper motor

    //Measure actual values when motor starts
    for (int i = 0; i < 800; i++) { //collect data from accelerometer
      BMI160.readAccelerometer(xAccelRaw, yAccelRaw, zAccelRaw);
    
      //convert raw data to g
      float xAccel = convertRawGyro(xAccelRaw);
      float yAccel = convertRawGyro(yAccelRaw);
      //Serial.println("MEASURED VALUES");
      //Serial.print(xAccel);Serial.print("\t");
      //Serial.print(yAccel);Serial.print("\t");  
      //Serial.println();
      //Calculate required values
      float avgXAccel = xAccel - (totalXAccelOffset/20); //average acceleration along x-axis after offset
      float avgYAccel = yAccel - (totalYAccelOffset/20); //average acceleration along y-axis after offset

      float netAccel = sqrt(sq(avgXAccel) + sq(avgYAccel)); // net acceleration

      float angularAccelRad = (netAccel * 9.807)/0.185; //angular acceleration is acceleration in g converted to m/s^2 divided by radius(dist from chip to center of top plate)

      float angularAccelDeg = (angularAccelRad/3.14159265359)*180;
  
      //Serial.println(angularAccelDeg);

      //Finding peak value
      if (peakValue < angularAccelDeg) {
        peakValue = angularAccelDeg;
      }
  
    }
  
    delay(5);
    analogWrite(stepPin,0); //turn off motor

    //Run peak value through Kalman filter
    peakValue = simpleKalmanFilter.updateEstimate(peakValue);
    
    Serial.print("Peak angular acceleration: ");
    Serial.println(peakValue);

    rslts[x] = peakValue;
  
    //reverse motor spinning direction and move motor back to starting position
    delay(1000);
    digitalWrite(dirPin, LOW); //Set spinning direction to CCW
    analogWrite(stepPin,127); //Run stepper motor
    delay(975);
    analogWrite(stepPin, 0); //Turn off motor
    delay(1000);

  }

  //Calculating mean peak value
  float totalPeakValue = 0;
  float meanPeakValue = 0;
  for (int y = 0; y < 10; y++) {
    //Serial.println(rslts[y]);
    totalPeakValue = totalPeakValue + rslts[y]; 
  }
  meanPeakValue = totalPeakValue/10;
  Serial.print("Mean: ");
  Serial.println(meanPeakValue);

  //Calculate standard deviation
  float sqDiffToMean = 0;
  float sumOfDiff = 0;
  float stanDev = 0;
  for (int z = 0; z < 10; z++) {
    sqDiffToMean = sq(meanPeakValue - rslts[z]);
    sumOfDiff =  sumOfDiff + sqDiffToMean;   
  }
  stanDev = sqrt(sumOfDiff/10);
  Serial.print("Standard deviation: ");
  Serial.println(stanDev);
  
  Serial.println("End of recording");
  delay(15000);


  //Reverse direction of motor
  //if (digitalRead(dirPin) == HIGH) {
   //digitalWrite(dirPin, LOW);
  //}
  //else {
   //digitalWrite(dirPin, HIGH);
  //}
  //delay(980); 

}




//Function to convert raw data from gyro------------------------------------------------------------
float convertRawGyro(int gRaw) {
  // since we are using 4 g range
  // -4 maps to a raw value of -32768
  // +4 maps to a raw value of 32767

  float g = (gRaw * 4) / 32768.0;

  return g;
}
