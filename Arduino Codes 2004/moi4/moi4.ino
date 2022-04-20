#include <Arduino_FreeRTOS.h> //FreeRTOS library
#include <BMI160Gen.h> //Gyro library
#include <SimpleKalmanFilter.h> //Kalman filter library
#include <AccelStepper.h> //AccelStepper library
//#include <queue.h>

//FreeRTOS definitions---------------------------------------------------------------------------
// define two tasks for Gyro & Motor
void TaskGyro( void *pvParameters );
void TaskMotor( void *pvParameters );
void TaskControl(void *pvParameters);

//define task handle
TaskHandle_t Gyro;
TaskHandle_t Motor;
TaskHandle_t Control;

//define queue handle
//QueueHandle_t accelQ;

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
#define motorInterfaceType 1
#define stepperMotorSignal 22

// Create a new instance of the AccelStepper class---------------------------------------------------------------------
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

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

  //Set sampling rate to 1600Hz
  BMI160.setAccelRate(12);
  
  //Calibrating accelerometer offset
  //Serial.println("Calibrating accelerometer");
  BMI160.autoCalibrateXAccelOffset(0);
  BMI160.setAccelOffsetEnabled(true);
  BMI160.autoCalibrateYAccelOffset(0);
  //BMI160.setAccelOffsetEnabled(true);
  //BMI160.autoCalibrateZAccelOffset(-1);
  BMI160.setAccelOffsetEnabled(true);
  
  //Serial.println("Accelerometer offset enabled");

//Set up motor------------------------------------------------------------------------------
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepperMotorSignal, OUTPUT);

  //change pwm frequency
  TCCR5B = TCCR5B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
  
  // Set the spinning direction to CCW:
  //digitalWrite(dirPin, LOW);

  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(20000);
  stepper.setAcceleration(8000);

  //Turn on power to stepper motor
  digitalWrite(stepperMotorSignal, LOW);

// Now set up two tasks to run independently----------------------------------------------------------------------------------------
  xTaskCreate(
    TaskGyro
    ,  "Gyro"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &Gyro );

  xTaskCreate(
    TaskMotor
    ,  "Motor"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &Motor);

   xTaskCreate(
    TaskControl
    ,  "Control"
    ,  128
    ,  NULL
    ,  2
    ,  &Control);

//Set up queue--------------------------------------------------------------------------------------------------------
//accelQ = xQueueCreate(200, sizeof(float));
    

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
  int loopCount = 0;
  float totalAccelDeg = 0;
  float avgAccelDeg = 0;
  

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
    angularAccelDeg = simpleKalmanFilter.updateEstimate(angularAccelDeg);
  
    //Serial.println(angularAccelDeg);

    totalAccelDeg = totalAccelDeg + angularAccelDeg;
    avgAccelDeg = totalAccelDeg / loopCount;
    Serial.println(avgAccelDeg);

    //Send angularAccelDeg to queue
    //xQueueSend(accelQ, &angularAccelDeg, portMAX_DELAY);
    
    vTaskDelay(1);
  }
}

void TaskMotor(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  float angularAccelDegQ;

  for (;;)
  {
    //resume Gyro task
    vTaskResume(Gyro);
    Serial.println("resume gyro");
    
    // Set the target position:
    stepper.moveTo(4000);
    //Run to target position with set speed and acceleration/deceleration:
    stepper.runToPosition();

    //suspend Gyro task
    vTaskSuspend(Gyro);
    Serial.println("suspend gyro");
    
    vTaskDelay(1000/portTICK_PERIOD_MS);

    //Receive angularAccelDeg from queue
    //for (int i = 0; i < 200; i++) {
      //if (xQueueReceive(accelQ, &angularAccelDegQ, portMAX_DELAY) == pdTRUE) {
        //Serial.println(angularAccelDegQ);
        
      //}
      //else {
        //Serial.println("fail to receive");
      //}
 
    //}

    // Move back to zero:
    stepper.moveTo(0);
    stepper.runToPosition(); 

    vTaskDelay(1000/portTICK_PERIOD_MS);

    //receive command from serial monitor
    if (Serial.available() > 0) {
      int incomingByte = Serial.parseInt();
        
      if (incomingByte == 12) { //send "12" to turn on power to stepper motor
        digitalWrite(stepperMotorSignal, LOW);
        Serial.print("Stepper Motor power on"); 
       }

      else if (incomingByte == 13) { //send "13" to turn off power to stepper motor
         digitalWrite(stepperMotorSignal, HIGH);
         Serial.print("Stepper Motor power off");
      }

      else if (incomingByte == 15) { //send "15" to suspend Gyro and Motor tasks and resume control task
        Serial.println("Suspend gyro and motor, resume control");
        vTaskSuspend(Gyro);
        vTaskResume(Control);
        vTaskSuspend(Motor);
      }
    }
  }
}

void TaskControl(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    //first suspend both Gyro and Motor tasks
    vTaskSuspend(Gyro);
    vTaskSuspend(Motor);
    Serial.println("Suspend motor and gyro");

    //take commands from serial monitor to resume Gyro and Motor tasks
    if (Serial.available() > 0) {
      int command = Serial.parseInt();

      if (command == 14) { //send "14" to resume Gyro and Motor tasks and suspend Control task
        Serial.println("Resume motor and gyro, suspend control");
        vTaskResume(Gyro);
        vTaskResume(Motor);      
        vTaskSuspend(Control);
      }
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
