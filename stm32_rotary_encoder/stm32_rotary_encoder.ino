#include <ros.h>
#include <std_msgs/String.h>

#define SIGNAL_A 2
#define SIGNAL_B 3

long long int  counter = 0;
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher encoder_counter("encoder_counter", &str_msg);
void setup()
{
  nh.initNode();
  nh.advertise(encoder_counter);
  pinMode(SIGNAL_A, INPUT_PULLUP);
  pinMode(SIGNAL_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_B, CHANGE);
}

void loop()
{
  char buf[100];
  sprintf(buf, "%lld", counter);
  str_msg.data = buf;
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
