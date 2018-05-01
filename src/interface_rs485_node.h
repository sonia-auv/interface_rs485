#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include "driver/serial.h"

#include <interface_rs485/SendRS485Msg.h>
#include <ros/ros.h>
#include <queue>
#include <string>
#include <thread>
#include <mutex>

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
        void readData(void);
        void writeData();

        void parseData();
        uint16_t calculateCheckSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, char* data);

        int writeCount = 0;
        int readCount = 0;

        ros::NodeHandlePtr nh;
        Serial serialConnection;
        std::thread reader;
        std::thread writer;
        std::thread parser;

        std::mutex writerMutex;
        std::mutex parserMutex;

        std::queue<SendRS485Msg::ConstPtr> writerQueue;
        std::queue<uint8_t> parseQueue;

        ros::Subscriber subscriber;
        ros::Publisher publisher;

        ros::Time timestamp;
    };
}

#endif
