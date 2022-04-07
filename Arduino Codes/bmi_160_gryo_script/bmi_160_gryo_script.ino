#include <BMI160Gen.h>

const int select_pin = 10;
const int i2c_addr = 0x69;
uint8_t OSR4 = OSR4;

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  //BMI160.begin(BMI160GenClass::SPI_MODE, select_pin);
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  // Set the accelerometer range to 250 degrees/second
  BMI160.setAccelerometerRange(2);
  Serial.println("Initializing IMU device...done.");

  //set filter
  BMI160.setAccelDLPFMode(OSR4);
}

void loop() {
  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  float gx, gy, gz;
  
  // read raw gyro measurements from device
  BMI160.readAccelerometer(gxRaw, gyRaw, gzRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);

  // display tab-separated gyro x/y/z values
  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();

  delay(500);
  
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 2) / 32768.0;

  return g;
}
