#ifndef Queue_Array_Sequential_NS
#define  Queue_Array_Sequential_NS

#include <iostream>
#include"ros/ros.h"
#include "std_msgs/Float32.h"
#include "beginner_tutorials/AddTwoInts.h"

namespace QueueArraySequentialNS{

class QueueArraySequential{
    private:
        int capacity, front, back;
        double *queue;
        void DoubleCapacity();

    public:
        QueueArraySequential(ros::NodeHandle& private_nh);
        ~QueueArraySequential();
        void dataCallback(const std_msgs::Float32::ConstPtr& msg);
        ros::NodeHandle& nodeHandle_;
        ros::Subscriber sub ;
        ros::ServiceServer service ;                                                                                                 
        bool server_read(beginner_tutorials::AddTwoInts::Request &req , beginner_tutorials::AddTwoInts::Response &res);
        void Push(double x);
        void Pop();                                         
        bool IsEmpty();                                                                                                                
        bool IsFull();                       
        double getFront();
        double getBack();
        int getSize();
        int getCapacity();    // 驗證用, 可有可無
    };

}
#endif //Queue_Array_Sequential_NS