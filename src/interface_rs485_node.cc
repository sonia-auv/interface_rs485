#include "interface_rs485_node.h"
#include <ros/ros.h>
#include <thread>
#include <mutex>


namespace interface_rs485
{

    // node Construtor
    InterfaceRs485Node::InterfaceRs485Node(const ros::NodeHandlePtr &_nh)
    : nh(_nh), serialConnection("/dev/ttyUSB0"),    reader(std::bind(&InterfaceRs485Node::readData, this)),
                                                    writer(std::bind(&InterfaceRs485Node::writeData, this)),
                                                    parser(std::bind(&InterfaceRs485Node::parseData, this))
    {
        publisher = nh->advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataTx", 100);
        subscriber = nh->subscribe("/interface_rs485/dataRx", 100, &InterfaceRs485Node::receiveData, this);

        reader.join();
        writer.join();
        parser.join();
    }

    // node destructor
    InterfaceRs485Node::~InterfaceRs485Node()
    {
        subscriber.shutdown();
    }

    // node spin
    void InterfaceRs485Node::Spin()
    {
        ros::Rate r(100);
        while(ros::ok())
        {
            ros::spinOnce();

            r.sleep();
        }
    }

    int InterfaceRs485Node::calculateCheckSum(unsigned char slave, unsigned char cmd, int nbByte, char* data)
    {
        int check = 0x3A+slave+cmd+nbByte+0x0D;
        for(int i = 0; i < nbByte; i++)
        {
            check += (unsigned char)data[i];
        }

        return check & 0xFFFF;
    }

    //callback when the subscriber receive data
    void InterfaceRs485Node::receiveData(const SendRS485Msg::ConstPtr &receivedData)
    {
        writerMutex.lock();
        writerQueue.push(receivedData);
        writerMutex.unlock();
    }

    // thread to read the data in the serial port and push it to the parseQueue
    void InterfaceRs485Node::readData()
    {
        while(!ros::isShuttingDown())
        {
            const char* data = serialConnection.receive().c_str();
            ROS_INFO("reading: %lu bytes", strlen(data));

            readCount++;
            if(readCount >= std::numeric_limits<int>::max() - 2)
            {
                readCount = 0;
            }
            for(unsigned int i = 0; i < strlen(data); i++)
            {
                parserMutex.lock();
                parseQueue.push(data[i]);
                parserMutex.unlock();
            }
        }
    }

    // thread to write the data in the serial port
    void InterfaceRs485Node::writeData()
    {
        while(!ros::isShuttingDown())
        {
            writerMutex.lock();
            int i = writerQueue.size();
            if(!writerQueue.empty())
            {
                SendRS485Msg::ConstPtr msg_ptr = writerQueue.front();
                writerQueue.pop();
                writerMutex.unlock();

                writeCount++;
                if(writeCount >= std::numeric_limits<int>::max() - 2)
                {
                    writeCount = 0;
                }

                int data_size = sizeof(msg_ptr->data) + 7;
                unsigned char data[data_size];
                data[0] = 0x3A;
                data[1] = msg_ptr->slave;
                data[2] = msg_ptr->cmd;
                data[3] = sizeof(msg_ptr->data);

                for(int i = 0; i < data[3]; i++)
                {
                    data[i+4] = msg_ptr->data[i];
                }

                int checksum = calculateCheckSum(data[1], data[2], data_size, (char*) &data[4]);

                data[data_size-2] = checksum >> 8;
                data[data_size-1] = checksum & 0xFF;
                data[data_size] = 0x0D;

                if(serialConnection.transmit((const char*)data) <= 0)
                {
                    ROS_INFO("RS485 send an empty packet...");
                }
            }
            else
            {
                writerMutex.unlock();
            }
        }
    }

    // thread to parse the data
    void InterfaceRs485Node::parseData()
    {
        while(!ros::isShuttingDown())
        {
            parserMutex.lock();
            if(parseQueue.size() >= 8)
            {
                //read until the start there or the queue is empty
                while(!parseQueue.empty()) {
                    if(parseQueue.front() != 0x3A)
                        parseQueue.pop();
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
                while(parseQueue.size() < nbByte);

                //temp variable to data
                char data_temp[nbByte];

                for(int i = 0; i < nbByte; i++)
                {
                    msg.data.push_back((unsigned char)parseQueue.front());
                    data_temp[i] = parseQueue.front();
                    parseQueue.pop();
                }

                int checksum = parseQueue.front() << 8;
                parseQueue.pop();

                checksum += parseQueue.front();
                parseQueue.pop();

                //pop the unused end data
                parseQueue.pop();

                parserMutex.unlock();

                int calc_checksum = calculateCheckSum(slave_temp, cmd_temp, nbByte, data_temp) & 0xFFFF;

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
