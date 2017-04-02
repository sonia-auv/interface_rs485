#!/usr/bin/env python

# (C) 2002-2006 Chris Liechti <cliechti@gmx.net>
# redirect data from a TCP/IP connection to a serial port and vice versa
# requires Python 2.2 'cause socket.sendall is used

# Modified version by Tennessee Carmel-Veilleux (May 9th 2008)
# Adds:
#    --file and --freq option for serial port playback from file to emulate
#    serial devices.
#
#    --sniff to log the input from the serial port to stdout

# import os
import socket
import threading

class ClientSocket:
    def __init__(self):
        self.s = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        # now connect to the web server on port 80
        # - the normal http port
        self.s.connect(('localhost', 9002))


        self.thread_read = threading.Thread(target=self.reader)
        self.thread_read.setDaemon(1)
        self.thread_read.start()

        while 1:
            cmd = raw_input('Enter data to send (slave;cmd;[data1,data2] :\n')
            cmd_tab = cmd.split(';')
            #uint8 SLAVE_ISI_0 = 16
            #uint8 SLAVE_ISI_1 = 17
            #uint8 SLAVE_ISI_2 = 18
            #uint8 SLAVE_ISI_3 = 19

            if cmd_tab[0] == '-1':
                self.sendCommand(16, cmd_tab[1],cmd_tab[2])
                self.sendCommand(17, cmd_tab[1],cmd_tab[2])
                self.sendCommand(18, cmd_tab[1],cmd_tab[2])
                self.sendCommand(19, cmd_tab[1],cmd_tab[2])
            else:
                self.sendCommand(cmd_tab[0], cmd_tab[1],cmd_tab[2])

    def sendCommand(self,slave,cmd,data):
        print 'sending ', '{}|{}|{}'.format(slave,cmd,data)
        self.s.sendall('{}|{}|{}'.format(slave,cmd,data))

        pass

    def reader(self):
        while 1:
            result = self.s.recv(2000)
            print str(result)

if __name__ == '__main__':
    ClientSocket()