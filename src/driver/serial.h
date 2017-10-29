//
// Created by dev on 10/26/17.
//

#ifndef INTERFACE_RS485_SERIAL_H
#define INTERFACE_RS485_SERIAL_H

#include <string>
#include <unistd.h>
#include <ros/ros.h>

class Serial{
public:
    Serial(std::string port);
    ~Serial();

    const char* receive(int length);
    int transmit(const char* data);

private:
    int fd;
};

#endif //INTERFACE_RS485_SERIAL_H
