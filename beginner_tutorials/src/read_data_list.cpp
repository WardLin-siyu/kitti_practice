#include"ros/ros.h"
#include "std_msgs/Float32.h"
#include "beginner_tutorials/AddTwoInts.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"read_data_list");
    if(argc !=3)
    {
        ROS_INFO("usage: you need to enter 1 2 to read data");
        return 1;
    }
    ros::NodeHandle n("send_data");
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("read_data_float");
    beginner_tutorials::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if(client.call(srv))
    {
        std::cout <<"Finsi read Bufer , Data = ";
        std::cout << srv.response.sum<< std::endl;
    }
    else
    {
        ROS_ERROR("Fail to read data");
    }


    return 0;
} 
