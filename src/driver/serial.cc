//
// Created by dev on 10/26/17.
//

#include "serial.h"
#include <fcntl.h>
#include <ros/ros.h>
#include <sys/ioctl.h>

Serial::Serial(std::string port)
{
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {
        ROS_ERROR("unable to connect to %s", port.c_str());
        ros::shutdown();
    }
    else
    {
        ROS_INFO("connection to %s succeed", port.c_str());
    }

    fcntl(fd, F_SETFL, O_NDELAY);

    tcgetattr(fd, &options);

    // setup le baudrate
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);

    tcsetattr(fd, TCSANOW, &options);
}

Serial::~Serial()
{
    close(fd);
}

std::string Serial::receive()
{
    ROS_DEBUG("interface_RS485 receive data");

    //blocking call
    char caracter;
    std::string data = "";
    while(read(fd, &caracter, 1) != -1)
    {
        data += caracter;
    }
    return data;
}

ssize_t Serial::transmit(const char* data, int string_length)
{
    ROS_DEBUG("interface_RS485 transmit data");
    return write(fd, data, string_length);
}
