#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include "driver/serial.h"

#include <sonia_common/SendRS485Msg.h>
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

        /// @brief Start the node.
        void Spin();

    private:
	Configuration configuration;

        // CheckSum comes from Manual
        uint16_t calculateCheckSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, std::vector<uint8_t> data);
        uint16_t calculateCheckSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, char* data);

        // Serial Stuff
        void receiveData(const sonia_common::SendRS485Msg::ConstPtr &receivedData);
        void readData();
        void writeData();
        void parseData();

        double sleepTime;

        ros::NodeHandlePtr nh;
        Serial serialConnection;
        std::thread reader;
        std::thread writer;
        std::thread parser;

        // [NJ] I think these are buffer queues
        SharedQueue<sonia_common::SendRS485Msg::ConstPtr> writerQueue;
        SharedQueue<uint8_t> parseQueue;

        ros::Subscriber subscriber;
        ros::Publisher publisher;
    };
}

#endif
