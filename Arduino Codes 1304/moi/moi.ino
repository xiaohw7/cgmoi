#include <Arduino_FreeRTOS.h> //FreeRTOS library
#include <BMI160Gen.h> //Gyro library
#include <SimpleKalmanFilter.h> //Kalman filter library

// define two tasks for Blink & AnalogRead
void TaskGyro( void *pvParameters );
void TaskMotor( void *pvParameters );

//Gyro definitions--------------------------------------------------------------------------------
const int select_pin = 10;
const int i2c_addr = 0x69;
uint8_t OSR4 = OSR4;

//define Kalman filter
SimpleKalmanFilter simpleKalmanFilter(12, 12, 0.5);

//Step motor definitions-----------------------------------------------------------------------------
// Define stepper motor connections:
#define dirPin 2
#define stepPin 4

// the setup function runs once when you press reset or power the board
void setup() {
//Set up Gyro----------------------------------------------------------------------------------------------------
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  // initialize device
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  // Set the accelerometer range to 4g
  BMI160.setAccelerometerRange(4);
  //Serial.println("Initializing IMU device...done.");

  //Set filter for gyro
  BMI160.setAccelDLPFMode(OSR4);

  //Calibrating accelerometer offset
  //Serial.println("Calibrating accelerometer");
  BMI160.autoCalibrateXAccelOffset(0);
  //BMI160.setAccelOffsetEnabled(true);
  BMI160.autoCalibrateYAccelOffset(0);
  //BMI160.setAccelOffsetEnabled(true);
  //BMI160.autoCalibrateZAccelOffset(-1);
  BMI160.setAccelOffsetEnabled(true);
  
  //Serial.println("Accelerometer offset enabled");

//Set up motor------------------------------------------------------------------------------
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  //change pwm frequency
  TCCR5B = TCCR5B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
  
  // Set the spinning direction to CCW:
  digitalWrite(dirPin, LOW);

// Now set up two tasks to run independently----------------------------------------------------------------------------------------
  xTaskCreate(
    TaskGyro
    ,  "Gyro"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskMotor
    ,  "Motor"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}


void TaskGyro(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  int xAccelRawOffset, yAccelRawOffset, zAccelRawOffset; //Raw offset data
  int xAccelRaw, yAccelRaw, zAccelRaw; //Raw actual data
  float totalYAccelOffset = 0; 
  float totalXAccelOffset = 0;

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

  for (;;) // A Task shall never return or exit.
  {
    BMI160.readAccelerometer(xAccelRaw, yAccelRaw, zAccelRaw);

    //convert raw data to g
    float xAccel = convertRawGyro(xAccelRaw);
    float yAccel = convertRawGyro(yAccelRaw);

    float netAccel = sqrt(sq(xAccel) + sq(yAccel)); // net acceleration

    netAccel = abs(netAccel - netOffset); //Minus off offset value

    float angularAccelRad = (netAccel * 9.807)/0.185; //angular acceleration is acceleration in g converted to m/s^2 divided by radius(dist from chip to center of top plate)

    //Serial.println(angularAccelRad);
    
    float angularAccelDeg = (angularAccelRad/3.14159265359)*180;

    //Run value through Kalman filter
    angularAccelDeg = simpleKalmanFilter.updateEstimate(angularAccelDeg);
  
    Serial.println(angularAccelDeg);
    
    vTaskDelay(1);
  }
}

void TaskMotor(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  

  for (;;)
  {
    //Set the spinning direction to CCW
    //change pwm frequency
    TCCR5B = TCCR5B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
    digitalWrite(dirPin, LOW);
    analogWrite(stepPin,127); //Run stepper motor
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    //slow down step motor by changing PWM frequency
    TCCR5B = TCCR5B & B11111000 | B00000011;  // for PWM frequency of   490.20 Hz
    delay(1000);
    analogWrite(stepPin,0); //Stop stepper motor
    //Set the spinning direction to CW
    //change pwm frequency
    TCCR5B = TCCR5B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
    digitalWrite(dirPin, HIGH);
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    analogWrite(stepPin,127); //Run stepper motor
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    //slow down step motor by changing PWM frequency
    TCCR5B = TCCR5B & B11111000 | B00000011;  // for PWM frequency of   490.20 Hz
    delay(1000);
    analogWrite(stepPin,0); //Stop stepper motor
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second

  
    
    //Serial.println("Sensor");
    //vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
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
