#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request &req , beginner_tutorials::AddTwoInts::Response &res)
{
    res.sum =req.a+req.b;
    ROS_INFO("request: (x=%lf) + (y=%lf) = %lf",req.a,req.b,res.sum);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"add_two_ints_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("add_two_ints",add);
    ROS_INFO("Ready to add two insts");
    ros::spin();    
    return 0;
} 
