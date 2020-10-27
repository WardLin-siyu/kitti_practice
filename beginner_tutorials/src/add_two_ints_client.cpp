#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>
int main(int argc,char **argv)
{
    ros::init(argc,argv,"add_two_ints_client");
    if(argc !=3)
    {
        ROS_INFO("usage: you need to add_two_client X Y");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
    beginner_tutorials::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if(client.call(srv))
    {
        std::cout << srv.response.sum;
    }
    else
    {
        ROS_ERROR("Fail ro call service  add_two_inst");
    }

    return 0;
}