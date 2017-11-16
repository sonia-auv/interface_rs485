#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include "driver/serial.h"

#include <interface_rs485/SendRS485Msg.h>
#include <ros/ros.h>
#include <queue>
#include <string>

namespace interface_rs485
{
    class InterfaceRs485Node
    {
    public:

        InterfaceRs485Node(const ros::NodeHandlePtr &_nh);
        ~InterfaceRs485Node();

        void Spin();

    private:

        void receiveData(const SendRS485Msg::ConstPtr &receivedData);
        void readData();
        void writeData();

        void parseData();
        int calculateCheckSum(unsigned char slave, unsigned char cmd, int nbByte, char* data);

        int writeCount = 0;
        int readCount = 0;

        ros::NodeHandlePtr nh;
        Serial serialConnection;

        std::queue<SendRS485Msg::ConstPtr> writerQueue;
        std::queue<unsigned char> parseQueue;

        ros::Subscriber subscriber;
        ros::Publisher publisher;

        ros::Time timestamp;
    };
}

#endif
