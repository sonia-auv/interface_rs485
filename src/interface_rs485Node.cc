#include "interface_rs485Node.h"
#include <thread>
#include <mutex>


namespace interface_rs485
{

    // node Construtor
    InterfaceRs485Node::InterfaceRs485Node(const ros::NodeHandlePtr &_nh) :
    nh(_nh)
    {
        serialConnection = Serial("/dev/ttyS5");
        publisher = nh->advertise<interface_rs485::SendRS485Msg>("/interface_rs485/dataTx", 100);
        subscriber = nh->subscribe("/interface_rs485/dataRx", 100, InterfaceRs485Node::receiveData);
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
            std::thread parser(InterfaceRs485Node::parseData);

            reader.join();
            writer.join();
            parser.join();

            r.sleep();
        }
    }

    // return a std::string to print
    std::string InterfaceRs485Node::printMsg(interface_rs485::ConstPtr &msg_ptr)
    {
        std::stringstream s;
        s << "slave: " << msg_ptr->slave << "/cmd: " << msg_ptr->cmd << "/data: " << msg_ptr->data << std::endl;
        return s.str();
    }

    //callback when the subscriber receive data
    void InterfaceRs485Node::receiveData(const interface_rs485::ConstPtr &receivedData)
    {
        writerQueue.push(receivedData);
    }

    // thread to read the data in the serial port and push it to the parseQueue
    void InterfaceRs485Node::readData()
    {
        while(!ros::isShuttingDown())
        {
            const char* data = serialConnection.receive();
            ROS_INFO("reading: %d bytes", strlen(data));

            readCount++;
            if(readCount >= std::numeric_limits<int>::max() - 2)
            {
                readCount = 0;
            }
            for(int i = 0; i < strlen(data); i++)
            {
                parseQueue.push(data[i]);
            }
        }
    }

    // thread to write the data in the serial port
    void InterfaceRs485Node::writeData()
    {
        while(!ros::isShuttingDown())
        {
            if(!writerQueue.empty())
            {
                interface_rs485::ConstPtr msg_ptr = writerQueue.pop();
                writeCount++;
                if(writeCount >= std::numeric_limits<int>::max() - 2)
                {
                    writeCount = 0;
                }

                int data_size = sizeof(msg_ptr->data) + 7;
                unsigned char data[data_size];
                data[0] = msg_ptr->start;
                data[1] = msg_ptr->slave;
                data[2] = msg_ptr->cmd;
                data[3] = sizeof(msg_ptr->data);

                for(int i = 0; i < data[3]; i++)
                {
                    data[i+4] = msg_ptr->data[i];
                }

                data[data_size-2] = msg_ptr->checksum >> 8;
                data[data_size-1] = msg_ptr->checksum & 0xFF;
                data[data_size] = msg_ptr->end;

                if(serialConnection.transmit(data) <= 0)
                {
                    ROS_INFO("RS485 send an empty packet...")
                }

                ROS_DEBUG("data in msg number %d: %s", writeCount, InterfaceRs485Node::printMsg(msg_ptr).c_str());
            }
        }
    }

    // thread to parse the data
    void InterfaceRs485Node::parseData()
    {
        while(!ros::isShuttingDown())
        {
            if(parseQueue.size() >= 8)
            {
                //read until the start there or the queue is empty
                for(char a = 0; !parseQueue.empty() && a != 0x3A; a = parseQueue.pop());
                SendRS485Msg msg = SendRS485Msg();
                msg.start = parseQueue.pop();
                msg.slave = parseQueue.pop();
                msg.cmd = parseQueue.pop();
                unsigned char nbByte = parseQueue.pop();
                unsigned char data[nbByte];

                // protection to prevent the buffer to read less byte
                while(parseQueue.size() < nbByte);

                int checksum = 0;
                for(int i = 0; i < nbByte; i++)
                {
                    data[i] = parseQueue.pop();
                    checksum += data[i];
                }

                msg.checksum = parseQueue.pop() << 8 | parseQueue.pop();
                msg.end = parseQueue.pop();

                checksum += (msg.start+msg.slave+msg.cmd);
                checksum &= 0xFFFF;

                // if the checksum is bad, drop the packet
                if(checksum == msg.checksum)
                {
                    publisher.publish(msg)
                }
            }
        }
    }
}
