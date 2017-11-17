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

    fcntl(fd, F_SETFL, 0);

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
    while(1)
    {
        int bytes;
        int length = ioctl(fd, FIONREAD, &bytes);

        if (length > 0)
        {
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
    }
}

int Serial::transmit(const char* data)
{
    ROS_DEBUG("interface_RS485 transmit data");
    return write(fd, data, strlen(data));
}
