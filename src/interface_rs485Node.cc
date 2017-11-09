#include "interface_rs485Node.h"
#include <ros/ros.h>
#include <thread>
#include <mutex>


namespace interface_rs485
{

    // node Construtor
    InterfaceRs485Node::InterfaceRs485Node(const ros::NodeHandlePtr &_nh) :
    nh(_nh)
    {
        serialConnection = Serial("/dev/ttyS5");
        ros::Publisher transmiter = nh->advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataTx", 100);
        ros::Suscriber receiver = nh->subscribe("/interface_rs485/dataRx", 100, InterfaceRs485Node::receiveData);
    }

    // node destructor
    InterfaceRs485Node::~InterfaceRs485Node()
    {
        receiver.shutdown();
    }

    // node spin
    void InterfaceRs485Node::Spin()
    {
        ros::Rate r(100);
        while(ros::ok())
        {
            ros::spinOnce();

            std::thread reader(InterfaceRs485Node::readData);
            std::thread writer(InterfaceRs485Node::writeData);

            reader.join();
            writer.join();

            r.sleep();
        }
    }

    void InterfaceRs485Node::transmitData()
    {
    }

    //callback when the subscriber receive data
    void InterfaceRs485Node::receiveData(const interface_rs485::ConstPtr &receivedData)
    {
        writerQueue.push(receivedData);
    }

    void InterfaceRs485Node::readData()
    {
        while(!ros::isShuttingDown())
        {

        }
    }

    // write the data in a message
    void InterfaceRs485Node::writeData()
    {
        while(!ros::isShuttingDown())
        {
            if(!writerQueue.empty())
            {
                interface_rs485::ConstPtr msg_ptr = writerQueue.pop();
                writeCount += 1;
                if(writeCount >= std::numeric_limits<int>::max() - 2)
                {
                    writeCount = 0;
                }

                ROS_DEBUG("data in msg number %d: %s", writeCount, InterfaceRs485Node::printMsg(msg_ptr).c_str());
            }
        }
    }

    // return a std::string to print
    std::string InterfaceRs485Node::printMsg(interface_rs485::ConstPtr &msg_ptr)
    {
        std::stringstream s;
        s << "slave: " << msg_ptr->slave << "/cmd: " << msg_ptr->cmd << "/data: " << msg_ptr->data << std::endl;
        return s.str();
    }
}
