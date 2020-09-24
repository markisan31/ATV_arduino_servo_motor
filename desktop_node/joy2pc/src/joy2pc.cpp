#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int16MultiArray.h"

ros::Publisher chatter_pub;


float mapf (float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void chatterCallback(const sensor_msgs::Joy& joyInfo)
{
  std_msgs::Int16MultiArray msg;
  ROS_INFO("sending data");
  for (int i = 0; i < 6; i++){
	
    msg.data.push_back((int)mapf(joyInfo.axes[i], -1, 1, 0, 1000));
}

  for (int i = 0; i < 25; i++){
	
    msg.data.push_back((int)map(joyInfo.buttons[i], 0, 1, 0, 1000));
}

  chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joy2serial");
  ros::NodeHandle n;
  
  
  ros::Subscriber sub = n.subscribe("joy", 1000, chatterCallback);
  chatter_pub = n.advertise<std_msgs::Int16MultiArray>("stearing", 1000);
      
    ros::spin();	

  return 0;
}





