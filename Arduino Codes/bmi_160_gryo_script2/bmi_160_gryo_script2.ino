#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;
void setup(){
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
}

void loop(){  
  int i = 0;
  int rslt;
  int16_t accelGyro[6]={0}; 
  float totalYAccel = 0;
  float totalXAccel = 0;
  for (int i = 0; i < 10; i++) {
    
    //get both accel and gyro data from bmi160
    //parameter accelGyro is the pointer to store the data
    rslt = bmi160.getAccelGyroData(accelGyro);
    float xAccel = accelGyro[3]/16384.0; //acceleration along x-axis
    float yAccel = accelGyro[4]/16384.0; //acceleration along y-axis
    //float zAccel = accelGyro[5]/16384.0; //acceleration along z-axis
    Serial.print(xAccel);Serial.print("\t");
    Serial.print(yAccel);Serial.print("\t"); 
    //Serial.print(zAccel);Serial.print("\t"); 
    Serial.println();
    totalXAccel = totalXAccel + xAccel;
    totalYAccel = totalYAccel + yAccel;
    Serial.print("total accel(x,y): ");
    Serial.print(totalXAccel);Serial.print("\t");
    Serial.println(totalYAccel);
    delay(10);
  }
  float avgXAccel = totalXAccel/10; //average acceleration along x-axis
  float avgYAccel = totalYAccel/10; //average acceleration along y-axis
  
  float netAccel = sqrt(sq(avgXAccel) + sq(avgYAccel)); // net acceleration
  Serial.print("Net Acceleration(g): ");
  Serial.println(netAccel);

  float angularAccelRad = (netAccel * 9.807)/0.185; //angular acceleration is acceleration converted to m/s^2 divided by radius(dist from chip to center of top plate)
  Serial.print("Angular acceleration(rad/s^2)");
  Serial.println(angularAccelRad);

  float angularAccelDeg = (angularAccelRad/3.14159265359)*180;
  Serial.print("Angular acceleration(deg/s^2)");
  Serial.println(angularAccelDeg);

  delay(5000);
  
  
  /*
   * //only read accel data from bmi160
   * int16_t onlyAccel[3]={0};
   * bmi160.getAccelData(onlyAccel);
   */

  /*
   * ////only read gyro data from bmi160
   * int16_t onlyGyro[3]={0};
   * bmi160.getGyroData(onlyGyro);
   */
}
