#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include "driver/serial.h"

#include <interface_rs485/SendRS485Msg.h>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <sharedQueue.h>
#include "Configuration.h"

namespace interface_rs485
{
    class InterfaceRs485Node
    {
    public:

        InterfaceRs485Node(const ros::NodeHandlePtr &_nh);
        ~InterfaceRs485Node();

        void Spin();

    private:
	Configuration configuration;

        uint16_t calculateCheckSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, std::vector<uint8_t> data);
        uint16_t calculateCheckSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, char* data);

        void receiveData(const SendRS485Msg::ConstPtr &receivedData);
        void readData();
        void writeData();
        void parseData();

        unsigned int writeCount = 0;
        unsigned int readCount = 0;

        ros::NodeHandlePtr nh;
        Serial serialConnection;
        std::thread reader;
        std::thread writer;
        std::thread parser;

        SharedQueue<SendRS485Msg::ConstPtr> writerQueue;
        SharedQueue<uint8_t> parseQueue;

        ros::Subscriber subscriber;
        ros::Publisher publisher;
    };
}

#endif
