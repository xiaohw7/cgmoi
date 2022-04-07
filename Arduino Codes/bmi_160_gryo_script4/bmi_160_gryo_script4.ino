/*
* The code is released under the GNU General Public License.
* Developed by www.codekraft.it 
*/

#include "CurieImu.h"

// IMU
float FS_SEL = 131;                                  // IMU gyro values to degrees/sec
unsigned long last_read_time;
float angle_x, angle_y, angle_z;                     // These are the result angles
float last_x_angle, last_y_angle, last_z_angle;      // These are the filtered angles
float lGXA, lGYA, lGZA;                              // Store the gyro angles to compare drift

// FUNCTIONS
//Math
//Convert radians to degrees
double rtod(double fradians) {return(fradians * 180.0 / PI);}

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x; last_y_angle = y; last_z_angle = z;
  lGXA = x_gyro; lGYA = y_gyro; lGZA = z_gyro;
}

void setup() {
  
  Serial.begin(9600);
  while (!Serial);

  // init CurieImu
  CurieImu.initialize();
  // use the code below to calibrate accel/gyro offset values
  Serial.println("Internal sensor offsets BEFORE calibration...");
  Serial.print(CurieImu.getXAccelOffset()); 
  Serial.print("\t"); // -76
  Serial.print(CurieImu.getYAccelOffset()); 
  Serial.print("\t"); // -235
  Serial.print(CurieImu.getZAccelOffset()); 
  Serial.print("\t"); // 168
  Serial.print(CurieImu.getXGyroOffset()); 
  Serial.print("\t"); // 0
  Serial.print(CurieImu.getYGyroOffset()); 
  Serial.print("\t"); // 0
  Serial.println(CurieImu.getZGyroOffset());
  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(1000);
  // The board must be resting in a horizontal position for 
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  CurieImu.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  CurieImu.autoCalibrateXAccelOffset(0);
  CurieImu.autoCalibrateYAccelOffset(0);
  CurieImu.autoCalibrateZAccelOffset(1);
  Serial.println(" Done");
  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieImu.setGyroOffsetEnabled(true);
  CurieImu.setAccelOffsetEnabled(true);

  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}
 
void loop() {
  unsigned long t_now = millis();
  int ax = CurieImu.getAccelerationX();
  int ay = CurieImu.getAccelerationY();
  int az = CurieImu.getAccelerationZ();
  int gx = CurieImu.getRotationX();
  int gy = CurieImu.getRotationY();
  int gz = CurieImu.getRotationZ();


  // Convert gyro values to degrees/sec
  float gyro_x = gx/FS_SEL;
  float gyro_y = gy/FS_SEL;
  float gyro_z = gz/FS_SEL;

   // Compute the accel angles
  float accelX = rtod(atan(ay / sqrt( pow(ax, 2) + pow(az, 2))));
  float accelY = rtod(-1 * atan(ax/sqrt(pow(ay,2) + pow(az,2))));
  
  // Compute the (filtered) gyro angles
  float dt =(t_now - last_read_time)/1000.0;
  float gyroX = gyro_x*dt + last_x_angle;
  float gyroY = gyro_y*dt + last_y_angle;
  float gyroZ = gyro_z*dt + last_z_angle;  

  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x*dt + lGXA;
  float unfiltered_gyro_angle_y = gyro_y*dt + lGYA;
  float unfiltered_gyro_angle_z = gyro_z*dt + lGZA;
  
  // Apply the complementary filter to figure out the change in angle 
  // Alpha depends on the sampling rate...
  float alpha = 0.96;
  angle_x = alpha*gyroX + (1.0 - alpha)*accelX;
  angle_y = alpha*gyroY + (1.0 - alpha)*accelY;
  angle_z = gyroZ;  //Accelerometer doesn't give z-angle

  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

  Serial.print("Y:" );
  Serial.print(angle_y);
  Serial.print("  \t Z:" );
  Serial.print(angle_z);
  Serial.print("  \t X:" );
  Serial.println(angle_x); 

}
