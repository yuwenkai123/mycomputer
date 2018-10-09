/*
mx_sensor_firmware with MPU6050 - Send sensor values from Sensor(arduino nano) board

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
#define MPU_INT 2 //MPU6050 INT Pin --> D2
MPU6050 accelgyro(0x68);

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

#define OUTPUT_MX_SENSORHUB
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

////////////////////////////////////////////////////////////////////////////////////////
//MPU6050 functions
void setup()
{

  //Init Serial port with 115200 baud rate
  Serial.begin(115200);

  //Setup MPU 6050
  Setup_MPU6050();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU6050 function
void Setup_MPU6050()
{


  Wire.begin();
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //Initialize DMP in MPU 6050
  Setup_MPU6050_DMP();

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup MPU 6050 DMP
void Setup_MPU6050_DMP()
{

  //DMP Initialization

  devStatus = accelgyro.dmpInitialize();

  //accelgyro.setXGyroOffset(220);
  //accelgyro.setXGyroOffset(76);
  //accelgyro.setXGyroOffset(-85);
  //accelgyro.setXGyroOffset(1788);

  if (devStatus == 0) {

    accelgyro.setDMPEnabled(true);

    pinMode(MPU_INT, INPUT_PULLUP);
    attachInterrupt(MPU_INT, dmpDataReady, RISING);

    mpuIntStatus = accelgyro.getIntStatus();

    dmpReady = true;

    packetSize = accelgyro.dmpGetFIFOPacketSize();

  } else {
    ;
  }


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP

void loop()
{

  //Read from Serial port
  Read_From_Serial();

  //Send MPU 6050 values through serial port
  Update_MPU6050();
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

  //DMP Processing

  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    ;
  }


  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();

  //get current FIFO count
  fifoCount = accelgyro.getFIFOCount();


  if ((mpuIntStatus & 0x10) || fifoCount > 512) {
    // reset so we can continue cleanly
    accelgyro.resetFIFO();
  }


  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_MX_SENSORHUB
    // display quaternion values in easy matrix form: w x y z
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    //accelgyro.dmpGetGravity(&gravity, &q);
    //accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);

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
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);

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
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&aa, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
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
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&aa, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

