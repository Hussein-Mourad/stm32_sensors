#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <ros.h>
#include <std_msgs/Float32.h>

#define TOPIC_NAME "yaw_kalman_filter"

HardwareSerial Serial3(PB11, PB10);
#define DEBUG

MPU6050 mpu; //0x68 address

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

ros::NodeHandle_<ArduinoHardware, 10, 10, 2048, 2048> nodeHandle;
std_msgs::Float32 yawAngle;
ros::Publisher publisher(TOPIC_NAME, &yawAngle);

void setup()
{
  (nodeHandle.getHardware())->setPort(&Serial3);
  (nodeHandle.getHardware())->setBaud(57600);
  nodeHandle.initNode();
  nodeHandle.advertise(publisher);

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

#ifdef DEBUG
  Serial3.begin(115200);
#endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);
    dmpReady = true;
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
#ifdef DEBUG
    Serial3.print(F("DMP Initialization failed (code "));
    Serial3.print(devStatus);
    Serial3.println(F(")"));
#endif
  }

}

void loop()
{
  if (!dmpReady)
    return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yawAngle.data = ypr[0] * 180 / M_PI;
    publisher.publish(&yawAngle);

#ifdef DEBUG
    Serial3.print("ypr\t");
    Serial3.print(ypr[0] * 180 / M_PI);
    Serial3.print("\t");
    Serial3.print(ypr[1] * 180 / M_PI);
    Serial3.print("\t");
    Serial3.println(ypr[2] * 180 / M_PI);
#endif

  }
  nodeHandle.spinOnce();
  delay(1000);
}
