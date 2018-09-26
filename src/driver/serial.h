//
// Created by dev on 10/26/17.
//

#ifndef INTERFACE_RS485_SERIAL_H
#define INTERFACE_RS485_SERIAL_H

#include <string>
#include <string.h>
#include <termios.h>
#include <unistd.h>


class Serial{
public:
    Serial(std::string port);
    ~Serial();

    ssize_t receive(char* data, size_t count);
    ssize_t transmit(const char* data, size_t string_length);

private:

    struct termios options;
    int fd;
};

#endif //INTERFACE_RS485_SERIAL_H
