#include <Arduino_FreeRTOS.h> //FreeRTOS library
#include <BMI160Gen.h> //Gyro library
#include <SimpleKalmanFilter.h> //Kalman filter library
#include <AccelStepper.h> //AccelStepper library

//FreeRTOS definitions---------------------------------------------------------------------------
// define two tasks for Gyro & Motor
void TaskGyro( void *pvParameters );
void TaskMotor( void *pvParameters );
void TaskControl(void *pvParameters);

//define task handle
TaskHandle_t Gyro;
TaskHandle_t Motor;
TaskHandle_t Control;

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

//define Kalman filter
SimpleKalmanFilter simpleKalmanFilter(1, 1, 0.5);

//Step motor definitions----------------------------------------------------------------------------------------------------------------------
// Define stepper motor connections:
#define dirPin 2
#define stepPin 4
#define motorInterfaceType 1
#define stepperMotorSignal 22

// Create a new instance of the AccelStepper class---------------------------------------------------------------------------------------------
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
  BMI160.setAccelOffsetEnabled(true);
  //BMI160.autoCalibrateZAccelOffset(-1);
  //BMI160.setAccelOffsetEnabled(true);
  
  //Serial.println("Accelerometer offset enabled");

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

//Set up motor------------------------------------------------------------------------------
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepperMotorSignal, OUTPUT);

  //change pwm frequency
  //TCCR5B = TCCR5B & B11111000 | B00000001;  // for PWM frequency of 31372.55 Hz
  
  // Set the spinning direction to CCW:
  //digitalWrite(dirPin, LOW);

  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(99999999);
  stepper.setAcceleration(999999);

  //Turn off power to stepper motor
  digitalWrite(stepperMotorSignal, HIGH);

// Now set up two tasks to run independently----------------------------------------------------------------------------------------
   xTaskCreate(
    TaskControl
    ,  "Control"
    ,  128
    ,  NULL
    ,  3
    ,  &Control);  
   
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



    //first suspend both Gyro and Motor tasks and turn off power to motor   
    vTaskSuspend(Gyro);    
    digitalWrite(stepperMotorSignal, HIGH);
    vTaskSuspend(Motor);
    Serial.println("Suspend motor and gyro, ready for input");

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

//Gyro task-----------------------------------------------------------------------------------------------------------------------------
void TaskGyro(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  for (;;) // A Task shall never return or exit.
  {
    if (loopCount < 30) { //Only record during acceleration process and neglecting the deceleration process
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

    
    //add angularAccelDeg to total tally and obtain average
    totalAccelDeg = totalAccelDeg + angularAccelDeg;
    avgAccelDeg = totalAccelDeg / loopCount;
    //Serial.println(avgAccelDeg);

   /* //get peak acceleration
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

//Motor task-----------------------------------------------------------------------------------------------------------
void TaskMotor(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  { 
    float rslts[10]; //array of 10 peak values from 10 measurements
    //Serial.println("Recording in progress...");

    //Record 10 values and add to rslts array. Since first 2 recordings tends to be inaccurate, i is set to 12 to so that the first 2 reading are not added to array
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

      //print average accel
      Serial.print("avg acceleration: ");
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
      totalAccelDeg = 0;
      avgAccelDeg = 0;
      peakAccel = 0;
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

    //Serial.println("Finish recording");
    vTaskDelay(5000/portTICK_PERIOD_MS);


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
        Serial.println("Suspend gyro and motor, resume control, ready for input");
        vTaskSuspend(Gyro);
        digitalWrite(stepperMotorSignal, HIGH);
        vTaskResume(Control);
        vTaskSuspend(Motor);
      }
    }
  }
}

//Control task---------------------------------------------------------------------------------------------------------------
void TaskControl(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    //first suspend both Gyro and Motor tasks and turn off power to motor   
    //vTaskSuspend(Gyro);    
    //digitalWrite(stepperMotorSignal, HIGH);
    //vTaskSuspend(Motor);
    //Serial.println("Suspend motor and gyro");

    //take commands from serial monitor to resume Gyro and Motor tasks
    if (Serial.available() > 0) {
      int command = Serial.parseInt();

      if (command == 14) { //send "14" to resume Gyro and Motor tasks and suspend Control task and turn on power to motor
        //Serial.println("Resume motor and gyro");
        digitalWrite(stepperMotorSignal, LOW); //turn on power to motor

        //reset values to 0
        loopCount = 0;
        //totalAccelDeg = 0;
        //avgAccelDeg = 0;
        //Serial.print("reset");
        
        vTaskResume(Motor);
        //TaskResume(Gyro);     
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
