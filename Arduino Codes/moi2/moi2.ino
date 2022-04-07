#include <DFRobot_BMI160.h> //Gyro library

//Gyro definitions----------------------------------------------------------------------------------
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

//Step motor definitions-----------------------------------------------------------------------------------------
// Define stepper motor connections:
#define dirPin 2
#define stepPin 4

void setup() {
//set up gyro--------------------------------------------------------------------------------------
  Serial.begin(115200);
  delay(100);
   //init the hardware bmin160  
  if (bmi160.softReset() != BMI160_OK){
    Serial.println("reset false");
    while(1);
  }
  //set and init the bmi160 i2c address
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("init false");
    while(1);
  }

//Set up motor--------------------------------------------------------------------------------
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  //change pwm frequency
  TCCR5B = TCCR5B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
  
  // Set the spinning direction CW/CCW:
  digitalWrite(dirPin, HIGH);
}

void loop() {
  //variables for gyro readings
  int rsltOffset; // results for offset readings
  int rslt; // results for actual reading
  int16_t accelGyroOffset[6]={0}; // offset data
  int16_t accelGyro[6]={0}; // actual data
  float totalYAccelOffset = 0; 
  float totalXAccelOffset = 0;
  float totalYAccel = 0;
  float totalXAccel = 0;

  //Measure offset values before motor starts
  for (int i = 0; i < 10; i++) { //collect data from accelerometer to calculate offset values
    //get both accel and gyro data from bmi160
    //parameter accelGyro is the pointer to store the data
    rsltOffset = bmi160.getAccelGyroData(accelGyro);
    float xAccelOffset = accelGyro[3]/16384.0; //acceleration along x-axis
    float yAccelOffset = accelGyro[4]/16384.0; //acceleration along y-axis
    Serial.println("OFFSET VALUES");
    Serial.print(xAccelOffset);Serial.print("\t");
    Serial.print(yAccelOffset);Serial.print("\t"); 
    Serial.println();
    totalXAccelOffset = totalXAccelOffset + xAccelOffset;
    totalYAccelOffset = totalYAccelOffset + yAccelOffset;
    Serial.print("total accel offset(x,y): ");
    Serial.print(totalXAccelOffset);Serial.print("\t");
    Serial.println(totalYAccelOffset);
    delay(10);
  }
  
  analogWrite(stepPin,127); //Run stepper motor
  delay(10);
  
  //Measure actual values when motor starts
  for (int i = 0; i < 10; i++) { //collect data from accelerometer
    //get both accel and gyro data from bmi160
    //parameter accelGyro is the pointer to store the data
    rslt = bmi160.getAccelGyroData(accelGyro);
    float xAccel = accelGyro[3]/16384.0; //acceleration along x-axis
    float yAccel = accelGyro[4]/16384.0; //acceleration along y-axis
    Serial.println("MEASURED VALUES");
    Serial.print(xAccel);Serial.print("\t");
    Serial.print(yAccel);Serial.print("\t");  
    Serial.println();
    totalXAccel = totalXAccel + xAccel;
    totalYAccel = totalYAccel + yAccel;
    Serial.print("total accel(x,y): ");
    Serial.print(totalXAccel);Serial.print("\t");
    Serial.println(totalYAccel);
    delay(5);
  }

  //calculate required values
  float avgXAccel = (totalXAccel - totalXAccelOffset)/10; //average acceleration along x-axis after offset
  float avgYAccel = (totalYAccel - totalYAccelOffset)/10; //average acceleration along y-axis after offset
  
  float netAccel = sqrt(sq(avgXAccel) + sq(avgYAccel)); // net acceleration
  Serial.print("Net Acceleration(g): ");
  Serial.println(netAccel);

  float angularAccelRad = (netAccel * 9.807)/0.185; //angular acceleration is acceleration in g converted to m/s^2 divided by radius(dist from chip to center of top plate)
  Serial.print("Angular acceleration(rad/s^2)");
  Serial.println(angularAccelRad);

  float angularAccelDeg = (angularAccelRad/3.14159265359)*180;
  Serial.print("Angular acceleration(deg/s^2)");
  Serial.println(angularAccelDeg);

  delay(500);
  analogWrite(stepPin,0); //turn off motor

  //reverse motor spinning direction
  if (digitalRead(dirPin) == HIGH) {
    digitalWrite(dirPin, LOW);
  }
  else {
    digitalWrite(dirPin, HIGH);
  }
  delay(3000);

}
