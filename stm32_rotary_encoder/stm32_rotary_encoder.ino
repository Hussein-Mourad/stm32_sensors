#include <ros.h>
#include <std_msgs/Int32.h>

#define SIGNAL_A PB12
#define SIGNAL_B PB13

#define TOPIC_NAME "encoder_count"

long  encoderCounter = 0;

ros::NodeHandle nh;
std_msgs::Int32 message;
ros::Publisher publisher(TOPIC_NAME, &message);

void setup()
{
  (nh.getHardware())->setPort(&Serial1);
  (nh.getHardware())->setBaud(115200);
  nh.initNode();
  nh.advertise(publisher);

  pinMode(SIGNAL_A, INPUT_PULLUP);
  pinMode(SIGNAL_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_B, CHANGE);
}

void loop()
{
  message.data = encoderCounter;
  publisher.publish(&message);
  nh.spinOnce();
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
