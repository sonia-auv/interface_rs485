#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include <interface_rs485/SendRS485Msg.h>
#include "driver/serial.h"
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

        void transmitData();
        void receiveData();
        void readData();
        void writeData();

        std::string printMsg();

        int writeCount = 0;
        int readCount = 0;

        ros::NodeHandlePtr nh;
        Serial serialConnection;

        std::queue<SendRS485Msg::ConstPtr> writerQueue;

        ros::Time timestamp;
    };
}

#endif
