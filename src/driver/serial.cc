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
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag &= ~(PARENB | PARODD);
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    //Input Flags
    options.c_iflag     &= ~IGNBRK;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    //Local Flags
    options.c_lflag  = 0;

    //Output Flags
    options.c_oflag  = 0;


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
