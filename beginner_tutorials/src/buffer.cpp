#include "buffer.h"
using::std::cout;
namespace QueueArraySequentialNS{

    QueueArraySequential::QueueArraySequential(ros::NodeHandle& private_nh):nodeHandle_(private_nh){
            capacity=5;
            queue = new double[capacity];
            sub = private_nh.subscribe<std_msgs::Float32>("float_data",10,&QueueArraySequential::dataCallback, this);
            service = private_nh.advertiseService("read_data_float",&QueueArraySequential::server_read, this);
            ROS_INFO("Ready to read data");

    }

    void QueueArraySequential::dataCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        Push(msg->data);
        std::cout <<"Buffer input : "<< getBack() <<std::endl; 
    }

    QueueArraySequential::~QueueArraySequential(){}

    bool QueueArraySequential::server_read(beginner_tutorials::AddTwoInts::Request &req , beginner_tutorials::AddTwoInts::Response &res)
    {
        if(req.a==1)
        {
            std::cout <<"Buffer Output:" << getFront() <<std::endl;
            res.sum =getFront(); 
            Pop();

        }
        else
        {
            ROS_INFO("I did't go msg");
        }
        return true;
    }
    void QueueArraySequential::DoubleCapacity(){
        capacity *= 2;
        double *newQueue = new double[capacity];

        int j = -1;
        for (int i = front+1; i <= back; i++) {
            j++;
            newQueue[j] = queue[i];
        }
        front = -1;       // 新的array從0開始, 把舊的array"整段平移", front跟back要更新
        back = j;
        delete [] queue;
        queue = newQueue;
    }

    void QueueArraySequential::Push(double x){

        if (IsFull()) {
            DoubleCapacity();
        }
        queue[++back] = x;
    }
    void QueueArraySequential::Pop(){

        if (IsEmpty()) {
            std::cout << "Queue is empty.\n";
            return;
        }
        front++;        
    }
    bool QueueArraySequential::IsFull(){

        return (back + 1 == capacity);
    }
    bool QueueArraySequential::IsEmpty(){

        return (front  == back);
    }
    double QueueArraySequential::getFront(){

        if (IsEmpty()) {
            std::cout << "Queue is empty.\n";
        }

        return queue[front+1];
    }
    double QueueArraySequential::getBack(){

        if (IsEmpty()) {
            std::cout << "Queue is empty.\n";
        }

        return queue[back];
    }
    int QueueArraySequential::getSize(){

        return (back - front);
    }
    int QueueArraySequential::getCapacity(){

        return capacity;
    }

    void printSequentialQueue (QueueArraySequential queue){
        cout << "front: " << queue.getFront() << "    back: " << queue.getBack() << "\n"
        << "capacity: " << queue.getCapacity() << "  number of elements: " << queue.getSize() << "\n\n";
    }
} 
