#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;    // set true if DMP init was successful
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;      // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z] quaternion container
VectorFloat gravity;  // [x, y, z] gravity vector
float ypr[3];         // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 172.0;
double setpoint = originalSetpoint;
double input, output;

//adjust these values to fit your own design
double Kp = 70;   //90;   //80;  //60;   //20    //25
double Kd = 1.6;  //2.5;  //2.3;  //2.3;  //1.0   //1.5
double Ki = 100;  //225;  //220;  //300;  //150   //250
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);


//-------------------------------------------------------------------
//Communication
int data0 = A0;
int data1 = A1;
int data2 = A2;
int dataValue = 0;
//-------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(9600);
  while (!Serial) { ; }

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(281);
  mpu.setYGyroOffset(-86);
  mpu.setZGyroOffset(-39);
  mpu.setXAccelOffset(2911);
  mpu.setYAccelOffset(-852);
  mpu.setZAccelOffset(323);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
  
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
  
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
  
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255); 
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //--------------------------------------------------------------------------
  //Communication
  pinMode(data0, INPUT);
  pinMode(data1, INPUT);
  pinMode(data2, INPUT);
  //--------------------------------------------------------------------------
}


void loop()
{
  //-------------------------------Communication------------------------------
  dataValue = 0;
  if(digitalRead(data0) == HIGH) dataValue = 1;
  if(digitalRead(data1) == HIGH) dataValue = dataValue + 2;
  if(digitalRead(data2) == HIGH) dataValue = dataValue + 4;
  //--------------------------------------------------------------------------

  //-----------------------------Movement Control-----------------------------
  switch(dataValue)
  {
    case 0: //p
      setpoint = originalSetpoint;
      break;
    case 1: //w
      setpoint = originalSetpoint - 1.0;
      break;
    case 2: //s
      setpoint = originalSetpoint + 2.0;
      break;
    case 3: //a
      setpoint = originalSetpoint - 2.0;
      break;
    case 4: //d
      setpoint = originalSetpoint - 2.0;
      break;
    case 5: //z
      setpoint = originalSetpoint + 2.0;
      break;
    case 6: //c
      setpoint = originalSetpoint + 2.0;
      break;
    case 7: //x
      setpoint = originalSetpoint;
      break;
  }
  //--------------------------------------------------------------------------


  //------------------------------Balance Control-----------------------------
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    movementControl();
  }
  movementControl();
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.resetFIFO();
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180/M_PI + 180;
  }
  //--------------------------------------------------------------------------
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
void movementControl()
{ 
  switch(dataValue)
  {
    case 0: //p
      //setpoint = originalSetpoint;
      motorController.move(output, MIN_ABS_SPEED, 0.0, 0.0);
      break;
    case 1: //w
      //setpoint = originalSetpoint - 1.5;
      motorController.move(output, MIN_ABS_SPEED, 1.0, 1.0);
      break;
    case 2: //s
      //setpoint = originalSetpoint + 1.0;
      motorController.move(output, MIN_ABS_SPEED, 1.0, 1.0);
      break;
    case 3: //a
      //setpoint = originalSetpoint - 1.5;
      motorController.move(output, MIN_ABS_SPEED, 1.0, 0.0);
      break;
    case 4: //d
      //setpoint = originalSetpoint - 1.5;
      motorController.move(output, MIN_ABS_SPEED, 0.0, 1.0);
      break;
    case 5: //z
      //setpoint = originalSetpoint + 1.0;
      motorController.move(output, MIN_ABS_SPEED, 1.0, 0.0);
      break;
    case 6: //c
      //setpoint = originalSetpoint + 1.0;
      motorController.move(output, MIN_ABS_SPEED, 0.0, 1.0);
      break;
    case 7: //x
      //setpoint = originalSetpoint;
      motorController.move(output, MIN_ABS_SPEED, 1.0, 1.0);
      break;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
