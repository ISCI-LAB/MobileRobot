/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;
long ansDouble;
std_msgs::Int32 output_msg;
ros::Publisher answer("/output", &output_msg);

void messageCb( const std_msgs::Int32& input_msg){
  long input = input_msg.data;
  ansDouble = input*2;
  output_msg.data = ansDouble;
  answer.publish( &output_msg);
}

ros::Subscriber<std_msgs::Int32> sub("/numbers", messageCb );

void setup()
{
  nh.initNode();
  nh.advertise(answer);
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(500);
}
