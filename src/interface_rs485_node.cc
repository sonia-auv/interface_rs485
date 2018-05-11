#include "interface_rs485_node.h"
#include <ros/ros.h>
#include <thread>
#include <mutex>


namespace interface_rs485
{

    // node Construtor
    InterfaceRs485Node::InterfaceRs485Node(const ros::NodeHandlePtr &_nh)
    : nh(_nh), serialConnection("/dev/ttyS5")
    {
        ROS_INFO("good");
        publisher = nh->advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataTx", 100);
        subscriber = nh->subscribe("/interface_rs485/dataRx", 100, &InterfaceRs485Node::receiveData, this);

        reader = std::thread(std::bind(&InterfaceRs485Node::readData, this));
        writer = std::thread(std::bind(&InterfaceRs485Node::writeData, this));
        parser = std::thread(std::bind(&InterfaceRs485Node::parseData, this));
    }

    // node destructor
    InterfaceRs485Node::~InterfaceRs485Node()
    {
        subscriber.shutdown();
    }

    // node spin
    void InterfaceRs485Node::Spin()
    {
        ros::Rate r(50);
        while(ros::ok())
        {
            ros::spinOnce();

            r.sleep();
        }
    }

    //calculate the checksum
    uint16_t InterfaceRs485Node::calculateCheckSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, char* data)
    {
        uint16_t check = (uint16_t)(0x3A+slave+cmd+nbByte+0x0D);
        for(uint8_t i = 0; i < nbByte; i++)
        {
            check += (uint8_t)data[i];
        }

        return check;
    }

    //callback when the subscriber receive data
    void InterfaceRs485Node::receiveData(const SendRS485Msg::ConstPtr &receivedData)
    {
        ROS_DEBUG("receive a rs485 data");
        writerMutex.lock();
        writerQueue.push(receivedData);
        writerMutex.unlock();
    }

    // thread to read the data in the serial port and push it to the parseQueue
    void InterfaceRs485Node::readData()
    {
        ROS_INFO("begin the read data threads");
        while(!ros::isShuttingDown())
        {
            ros::Duration(0.01).sleep();
            std::string data = serialConnection.receive();
            readCount++;
            if(readCount >= std::numeric_limits<int>::max() - 2)
            {
                readCount = 0;
            }

            for(unsigned int i = 0; i < data.size(); i++)
            {
                parserMutex.lock();
                parseQueue.push((uint8_t)data[i]);
                parserMutex.unlock();
            }
        }
    }

    // thread to write the data in the serial port
    void InterfaceRs485Node::writeData()
    {
        ROS_INFO("begin the write data threads");
        while(!ros::isShuttingDown())
        {
            ros::Duration(0.01).sleep();
            writerMutex.lock();
            while(!writerQueue.empty())
            {
                SendRS485Msg::ConstPtr msg_ptr = writerQueue.front();
                writerQueue.pop();
                writerMutex.unlock();

                writeCount++;
                if(writeCount >= std::numeric_limits<int>::max() - 2)
                {
                    writeCount = 0;
                }

                unsigned long data_size = msg_ptr->data.size() + 7;
                uint8_t data[data_size];
                data[0] = 0x3A;
                data[1] = msg_ptr->slave;
                data[2] = msg_ptr->cmd;
                data[3] = (uint8_t)msg_ptr->data.size();

                for(int i = 0; i < data[3]; i++)
                {
                    data[i+4] = msg_ptr->data[i];
                }

                uint16_t checksum = calculateCheckSum(data[1], data[2], data[3], (char*) &data[4]);

                data[data_size-3] = (uint8_t)(checksum >> 8);
                data[data_size-2] = (uint8_t)(checksum & 0xFF);
                data[data_size-1] = 0x0D;

                ROS_DEBUG("%0x\n%0x\n%0x\n%0x\n%0x\n%0x\n%0x\n%0x\n", data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

                if(serialConnection.transmit((const char*)data, data_size) <= 0)
                {
                    ROS_INFO("RS485 send an empty packet...");
                }
            }
            writerMutex.unlock();
        }
    }

    // thread to parse the data
    void InterfaceRs485Node::parseData()
    {
        ROS_INFO("begin the parse data threads");
        while(!ros::isShuttingDown())
        {
            ros::Duration(0.01).sleep();
            parserMutex.lock();
            if(parseQueue.size() >= 8)
            {
                //read until the start there or the queue is empty
                while(!parseQueue.empty()) {
                    if(parseQueue.front() != 0x3A)
                    {
                        parseQueue.pop();
                    }
                    else
                    {
                        break;
                    }
                }
                if(parseQueue.empty())
                {
                    continue;
                }

                SendRS485Msg msg = SendRS485Msg();

                //pop the unused start data
                parseQueue.pop();

                //temp variable to slave
                unsigned char slave_temp = parseQueue.front();
                msg.slave = parseQueue.front();
                parseQueue.pop();

                // temp variable to cmd
                unsigned char cmd_temp = parseQueue.front();
                msg.cmd = parseQueue.front();
                parseQueue.pop();

                unsigned char nbByte = parseQueue.front();
                parseQueue.pop();

                // protection to prevent the buffer to read less byte
                parserMutex.unlock();
                while(parseQueue.size() < nbByte);

                parserMutex.lock();

                //temp variable to data
                char data_temp[nbByte];

                for(int i = 0; i < nbByte; i++)
                {
                    msg.data.push_back((unsigned char)parseQueue.front());
                    data_temp[i] = parseQueue.front();
                    parseQueue.pop();
                }

                uint16_t checksum = (uint16_t)(parseQueue.front()<<8);
                parseQueue.pop();

                checksum += parseQueue.front();
                parseQueue.pop();

                //pop the unused end data
                parseQueue.pop();

                parserMutex.unlock();

                uint16_t calc_checksum = calculateCheckSum(slave_temp, cmd_temp, nbByte, data_temp);

                // if the checksum is bad, drop the packet
                if(checksum == calc_checksum)
                {
                    publisher.publish(msg);
                }
            }
            else
            {
                parserMutex.unlock();
            }
        }
    }
}
