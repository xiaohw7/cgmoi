#include <BMI160Gen.h>
#include <SimpleKalmanFilter.h>

const int select_pin = 10;
const int i2c_addr = 0x69;
uint8_t OSR2 = OSR2;

SimpleKalmanFilter simpleKalmanFilter(0.01, 0.01, 0.05);

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  // Set the accelerometer range to 2g
  BMI160.setAccelerometerRange(2);
  //Serial.println("Initializing IMU device...done.");

  //set filter
  BMI160.setAccelDLPFMode(OSR2);

  //set sampling rate
  //BMI160.setAccelRate(12);
  

  //Calibrating accelerometer offset
  //Serial.println("Calibrating accelerometer");
  BMI160.autoCalibrateXAccelOffset(0);
  BMI160.setAccelOffsetEnabled(true);
  BMI160.autoCalibrateYAccelOffset(0);
  //BMI160.setAccelOffsetEnabled(true);
  //BMI160.autoCalibrateZAccelOffset(-1);
  BMI160.setAccelOffsetEnabled(true);
  //delay(1000);
  
  delay(500);
  //Serial.println("Accelerometer offset enabled");

}

void loop() {
  int gxRaw1, gyRaw1, gzRaw1, gxRaw2, gyRaw2, gzRaw2;// raw gyro values
  float gx1, gy1, gz1, gx2, gy2, gz2;
  int x, y, z;
    // read raw gyro measurements from device
    BMI160.readAccelerometer(gxRaw1, gyRaw1, gzRaw1); // first reading
    //BMI160.getAcceleration(x, y, z);
    
    //gxRaw1 = simpleKalmanFilter.updateEstimate(gxRaw1);
    //gyRaw1 = simpleKalmanFilter.updateEstimate(gyRaw1);
    //gzRaw1 = simpleKalmanFilter.updateEstimate(gzRaw1);
    
    //Serial.print(gxRaw1);Serial.print("\t");
    //Serial.print(gyRaw1);Serial.print("\t");
    //Serial.println(gzRaw1);
    //convert the raw gyro data to g
    gx1 = convertRawGyro(gxRaw1);
    gy1 = convertRawGyro(gyRaw1);
    gz1 = convertRawGyro(gzRaw1);
    //convert to metres/second
    gx1 = gx1*9.81;
    gy1 = gy1*9.81;
    gz1 = gz1*9.81;

    float netaccel = sqrt(sq(gx1)+sq(gy1));
    float angaccel = (netaccel/(0.185*3.14))*180;
    Serial.println(angaccel);
    //Serial.print("gx1 (g): ");
    //Serial.print(gx1); Serial.print("\t");
    //Serial.print("gy1 (g): ");
    //Serial.println(gy1); Serial.print("\t");
    //Serial.print("gz1 (g): ");
    //Serial.println(gz1);
    //delay(500);


}
float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 2) / 32768.0;

  return g;
}
