/*
  mx_sensor_firmware - Send sensor values from Sensor(arduino nano) board

  Created September 2018

  Copyright(c) 2018 sunMaxwell
*/

//MPU 6050 Interfacing libraries

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"

//Contain definition of maximum limits of various data type
#include <limits.h>

//Creating MPU6050 Object
#define MPU_INT 0 //MPU6050 INT Pin --> D2
MPU6050 mpu(0x68);

///////////////////////////////////////////////////////////////////////////////////////
//DMP options
//Set true if DMP init was successful
bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP paclet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];

//Defining an LED pin to show the status of IMU data read
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

//#define DEBUG
#define PING_NUM 6     // Ultrasonic Sensor Number
#define RATE_DELAY 10  // ms
#define IMU_MPU9250    // IMU_MPU6050
#define OUTPUT_IMU     // USE IMU Message
////////////////////////////////////////////////////////////////////////////////////////////////
//orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//Ultrasonic pins definition
const int Trig[PING_NUM] = {4, 6, 8, 10, 14, 16};
const int Echo[PING_NUM] = {5, 7, 9, 11, 15, 17};
////////////////////////////////////////////////////////////////////////////////////////
//MPU6050 functions
void setup()
{

  //Init Serial port with 115200 baud rate
  Serial.begin(115200);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  //Setup Ultrasonic
  SetupUltrasonic();

  //Setup MPU 6050
  Setup_MPU6050();

}

//Setup UltrasonicsSensor() function
void SetupUltrasonic()
{
  uint8_t i = 0;
  for (i = 0; i < PING_NUM; i++)
  {
    pinMode(Trig[i], OUTPUT);
    pinMode(Echo[i], INPUT);
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU6050 function
void Setup_MPU6050()
{
  Wire.begin();
#ifdef DEBUG
  // initialize device
  Serial.println("Initializing I2C devices...");
#endif
  mpu.initialize();

#ifdef DEBUG
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
#endif
  //Initialize DMP in MPU 6050
  Setup_MPU6050_DMP();

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU 6050 DMP
void Setup_MPU6050_DMP()
{

  //DMP Initialization

  devStatus = mpu.dmpInitialize();

  //mpu.setXGyroOffset(220);
  //mpu.setXGyroOffset(76);
  //mpu.setXGyroOffset(-85);
  //mpu.setXGyroOffset(1788);

  if (devStatus == 0) {

    mpu.setDMPEnabled(true);

    pinMode(MPU_INT, INPUT_PULLUP);
    attachInterrupt(MPU_INT, dmpDataReady, RISING);

    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {
    ;
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP

void loop()
{

  //Read from Serial port
  //Read_From_Serial();

  //Send MPU 6050 values through serial port
  Update_MPU6050();

  //Send ultrasonic values through serial port
  Update_Ultra_Sonic();

  //Send Freq. ADJ
  Update_Rate(RATE_DELAY);

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Update_Rate(uint8_t ms)
{
  delay(ms);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update ultrasonic sensors through serial port

long Read_Ultra_Sonic(uint8_t Trig, uint8_t Echo)
{
  long duration, cm;

  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(Echo, HIGH, 10000);
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  return cm;
}

void Update_Ultra_Sonic()
{
  uint8_t i;
  long cm[6];
  for (i = 0; i < PING_NUM; i++)
  {
    cm[i] = Read_Ultra_Sonic(Trig[i], Echo[i]);
  }

  //Sending through serial port
  Serial.print("u");
  Serial.print("\t");
  for (i = 0; i < PING_NUM; i++)
  {
    Serial.print(cm[i]);
    if (i < PING_NUM - 1)
    {
      Serial.print("\t");
    }
  }

  Serial.print("\n");

}


//Conversion of microsecond to centimeter
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial()

{
  while (Serial.available() > 0)
  {
    int data = Serial.read();
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050

void Update_MPU6050()
{

  ///Update values from DMP for getting rotation vector
  Update_MPU6050_DMP();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050 DMP functions

void Update_MPU6050_DMP()
{
  uint8_t mpuStatus = 0x01;//default mpu9250 mpuIntStatus
  uint16_t fifoSize = 1024;//default mpu9250 fifoSize

  //DMP Processing

  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    ;
  }


  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  //get current FIFO count
  fifoCount = mpu.getFIFOCount();

#ifdef IMU_MPU9250
  fifoSize = 1024;
  mpuStatus = 0x01;
#endif

#ifdef IMU_MPU6050
  fifoSize = 512;
  mpuStatus = 0x02;
#endif

  if ((mpuIntStatus & 0x10) || fifoCount > fifoSize) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
  }

  else if (mpuIntStatus & mpuStatus) {

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_IMU
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetGravity(&gravity, &q);
    //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Serial.print("i"); Serial.print("\t");
    Serial.print(q.x); Serial.print("\t");
    Serial.print(q.y); Serial.print("\t");
    Serial.print(q.z); Serial.print("\t");
    Serial.print(q.w); //Serial.print("\t");
    /*
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[2] * 180 / M_PI);
        Serial.print("\t");
    */
    Serial.print("\n");
#endif

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    Serial.print("quaternion\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.print(q.z);
    Serial.print("\t");
    Serial.println(q.w);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
  }

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

