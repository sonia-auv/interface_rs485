//
// Created by dev on 10/26/17.
//

#include "serial.h"

Serial::Serial(std::string port)
{
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(tty == -1)
    {
        ROS_ERROR("unable to connect to %s", port.c_str());
    }
    else
    {
        ROS_INFO("connection to %s succeed", port.c_str())
    }
}

Serial::~Serial()
{
    close(fd);
}

const char* Serial::receive(int length)
{
    ROS_DEBUG("interface_RS485 receive data")
    char data[length];
    if(read(fd, &data, sizeof(data)) != -1)
    {
        return data;
    }
    else
    {
        return "";
    }
}

int Serial::transmit(const char* data)
{
    ROS_DEBUG("interface_RS485 transmit data")
    return write(fd, data, strlen(data));
}
