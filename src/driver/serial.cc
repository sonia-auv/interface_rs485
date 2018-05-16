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

ssize_t Serial::receive(char* data, size_t count)
{
    ROS_DEBUG("interface_RS485 receive data");

    return read(fd, data, count);
}

ssize_t Serial::transmit(const char* data, size_t string_length)
{
    ROS_DEBUG("interface_RS485 transmit data");
    return write(fd, data, string_length);
}
