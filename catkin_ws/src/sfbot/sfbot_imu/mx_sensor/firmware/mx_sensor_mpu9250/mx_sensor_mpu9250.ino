// These are the Arduino headers to read IMU values using I2C protocoal.
//It also include Arduino - MPU 9150 headers for performing special functions.

#include "Wire.h"
//#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Creating a MPU6050 handle which can be used for MPU 9250
MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//VectorInt16 AccelRaw;
//VectorInt16 AccelLinear;
//int32_t Gyro[3];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
// This is the setup function of Arduino
// This will initialize I2C communication, ROS node handle, TF publisher, ROS publisher and DMP of IMU
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  Serial.begin(115200);

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {

    ;
  }

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything

  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    ;
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();


    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x01) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetGravity(&gravity, &q);
    //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //mpu.dmpGetGyro(Gyro, fifoBuffer);
    //mpu.dmpGetAccel(&AccelRaw, fifoBuffer);
    //mpu.dmpGetLinearAccel(&AccelLinear, &AccelRaw, &gravity);

    Serial.print("i"); Serial.print("\t");
    Serial.print(q.x); Serial.print("\t");
    Serial.print(q.y); Serial.print("\t");
    Serial.print(q.z); Serial.print("\t");
    Serial.print(q.w); Serial.print("\t");
    /*
            Serial.print(Gyro[0]/16384.0/57.3); Serial.print("\t");
            Serial.print(Gyro[1]/16384.0/57.3); Serial.print("\t");
            Serial.print(Gyro[2]/16384.0/57.3); Serial.print("\t");

            Serial.print(AccelLinear.x/4096.0); Serial.print("\t");
            Serial.print(AccelLinear.y/4096.0); Serial.print("\t");
            Serial.print(AccelLinear.z/4096.0);
    */
    Serial.print("\n");
  }
}
