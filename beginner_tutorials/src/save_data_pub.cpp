#include"ros/ros.h"
#include "std_msgs/Float32.h"
int main(int argc, char **argv)
{
  ros::init(argc,argv,"send_data");
  ros::NodeHandle n("~");
  ros::Publisher float_data_pub = n.advertise<std_msgs::Float32>("float_data",10);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    std_msgs::Float32 msg;
    msg.data=0;
    std::cin >> msg.data;
    ROS_INFO("%f",msg.data);
    float_data_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}