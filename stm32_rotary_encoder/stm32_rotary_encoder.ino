#include <ros.h>
#include <std_msgs/Int32.h>

#define SIGNAL_A PB12
#define SIGNAL_B PB13

long  counter = 0;
ros::NodeHandle nh;
std_msgs::Int32 str_msg;
ros::Publisher encoder_counter("encoder_counter", &str_msg);
void setup()
{
  (nh.getHardware())->setPort(&Serial1);
  (nh.getHardware())->setBaud(115200);
  nh.initNode();
  nh.advertise(encoder_counter);
  pinMode(SIGNAL_A, INPUT_PULLUP);
  pinMode(SIGNAL_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_B, CHANGE);
}

void loop()
{
//  char buf[100];
//  sprintf(buf, "%lld", counter);
//  str_msg.data = buf;
  
  str_msg.data = counter;
  encoder_counter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}

void ISR_A()
{
  if (digitalRead(SIGNAL_A) != digitalRead(SIGNAL_B))
  {
    counter++;
  }
  else
  {
    counter--;
  }
}

void ISR_B()
{
  if (digitalRead(SIGNAL_A) == digitalRead(SIGNAL_B))
  {
    counter++;
  }
  else
  {
    counter--;
  }
}
