#include <ros.h>
#include <std_msgs/Int32.h>

#define SIGNAL_A PB3
#define SIGNAL_B PA15

#define TOPIC_NAME "encoder_count"
HardwareSerial Serial3(PB11,PB10);

long  encoderCounter = 0;

// ros::NodeHandle nh;
// std_msgs::Int32 message;
// ros::Publisher publisher(TOPIC_NAME, &message);

void setup()
{
  Serial3.begin(115200);
  //(nh.getHardware())->setPort(&Serial1);
  //(nh.getHardware())->setBaud(115200);
  //nh.initNode();
  //nh.advertise(publisher);

  pinMode(SIGNAL_A, INPUT_PULLUP);
  pinMode(SIGNAL_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_B, CHANGE);
}

void loop()
{
  //message.data = encoderCounter;
 // publisher.publish(&message);
//  nh.spinOnce();
  Serial3.println(encoderCounter);
  delay(1000);
}

void ISR_A()
{
  digitalRead(SIGNAL_A) != digitalRead(SIGNAL_B) ? encoderCounter++ : encoderCounter--;

}

void ISR_B()
{
  digitalRead(SIGNAL_A) == digitalRead(SIGNAL_B) ? encoderCounter++ : encoderCounter--;
}
