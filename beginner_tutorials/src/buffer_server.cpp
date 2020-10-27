#include"ros/ros.h"
#include "std_msgs/Float32.h"
//#include "beginner_tutorials/AddTwoInts.h"
#include "buffer.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "read_data");
    ros::NodeHandle private_nh("send_data");
    //ros::Subscriber sub = n.subscribe("float_data",10,dataCallback);
    QueueArraySequentialNS::QueueArraySequential QueueArraySequ(private_nh);
    //ros::ServiceServer service = n.advertiseService("read_data_float",server_read);
    ros::spin();
    return 0;
} 
