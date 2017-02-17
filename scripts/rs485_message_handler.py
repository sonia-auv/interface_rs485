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

import os
import serial
import socket
import sys
import threading
import time

import rospy
from Queue import Queue
from optparse import (OptionParser)
from sonia_msgs.msg import SendRS485Msg

import rospy

class RS485MessageHandler:
    def __init__(self, serial, sniff):
        self.serial = serial
        self.socket = socket
        self.sniff = sniff
        self.count_read_data = 0
        self.max_read_data_print = 200
        self.count_write_data = 0
        self.max_write_data_print = 200
        self.queue_to_write = Queue()

    def initializer(self):
        self.thread_read = threading.Thread(target=self.reader)
        self.thread_read.setDaemon(1)
        self.thread_read.start()
        sub = rospy.Subscriber('/interface_rs485/SendRS485Msg', SendRS485Msg, self.sendRS485_message_received())
        self.writer() #This is blocking
        sub.unregister()

    def reader(self):
        """loop forever and copy serial->socket"""
        while not rospy.is_shutdown():
            try:
                data = self.serial.read(1)  # read one, blocking
                n = self.serial.inWaiting()  # look if there is more
                if n:
                    data = data + self.serial.read(
                        n)  # and get as much as possible

                if data:
                    # Pipe to stdout if sniffing
                    if self.sniff:
                        print('Read:',data)
                        sys.stdout.flush()

                    self.count_read_data += 1
                    if not (self.count_read_data % self.max_read_data_print):
                        print("read %d data." % self.count_read_data)
                        sys.stdout.flush()
                    if self.count_read_data >= sys.maxint - 2:
                        self.count_read_data = 0

                    self.new_data_read_from_rs485(data)
            except IOError, msg:
                print(msg)
                break


    def writer(self):
        """loop forever"""
        while not rospy.is_shutdown():
            try:
                send_rs485_msg = self.queue_to_write.get() # this is blocking.
                if send_rs485_msg:
                    # Pipe to stdout if sniffing
                    if self.sniff:
                        print('Write:',send_rs485_msg)
                        sys.stdout.flush()

                    self.count_read_data += 1
                    if not (self.count_read_data % self.max_read_data_print):
                        print("write %d data." % self.count_read_data)
                        sys.stdout.flush()
                    if self.count_read_data >= sys.maxint - 2:
                        self.count_read_data = 0

                    self.write_data_to_rs485(send_rs485_msg)
            except IOError, send_rs485_msg:
                print(send_rs485_msg)
                break

    def sendRS485_message_received(self, send_rs485_msg):
        self.queue_to_write.put(send_rs485_msg)


    def write_data_to_rs485(self, send_rs485_msg):
        # TODO: Parse message and send to serial
        slave = send_rs485_msg.slave # unsigned byte
        cmd = send_rs485_msg.cmd # unsigned byte
        data = send_rs485_msg.data # byte array

        # self.serial.write(data)
        pass

    def new_data_read_from_rs485(self,data):
        # TODO : Parse data and publish it.
        # data is bytes. accessible via data[0], data[1]
        pass

class PassThroughOptionParser(OptionParser):
    def error(self, msg):
        pass


def initialize_options():
    global parser, options, args
    parser = PassThroughOptionParser(usage="""\
        %prog [options] [port [baudrate]]
        Simple Serial RS485 interface.
        """)
    parser.add_option("-p", "--port", dest="port",
                      help="port, a number (default 0) or a device name (deprecated option)",
                      default=None)
    parser.add_option("-b", "--baud", dest="baudrate", action="store",
                      type='int',
                      help="set baudrate, default 9600", default=9600)
    parser.add_option("", "--parity", dest="parity", action="store",
                      help="set parity, one of [N, E, O], default=N",
                      default='N')
    parser.add_option("", "--rtscts", dest="rtscts", action="store_true",
                      help="enable RTS/CTS flow control (default off)",
                      default=False)
    parser.add_option("", "--xonxoff", dest="xonxoff", action="store_true",
                      help="enable software flow control (default off)",
                      default=False)
    parser.add_option("", "--cr", dest="cr", action="store_true",
                      help="do not send CR+LF, send CR only", default=False)
    parser.add_option("", "--lf", dest="lf", action="store_true",
                      help="do not send CR+LF, send LF only", default=False)
    parser.add_option("", "--rts", dest="rts_state", action="store", type='int',
                      help="set initial RTS line state (possible values: 0, 1)",
                      default=None)
    parser.add_option("", "--dtr", dest="dtr_state", action="store", type='int',
                      help="set initial DTR line state (possible values: 0, 1)",
                      default=None)
    parser.add_option("-q", "--quiet", dest="quiet", action="store_true",
                      help="suppress non error messages", default=False)
    parser.add_option("-s", "--sniff", dest="sniff", action="store_true",
                      help="output read data to stdout", default=False)
    # Parsing only known arguments
    (options, args) = parser.parse_args()


def assert_options_are_valid():
    global baudrate, port
    if args:
        if options.port is not None:
            parser.error(
                "no arguments are allowed, options only when --port is given")
        args.pop(0)
        if args:
            try:
                baudrate = int(args[0])
            except ValueError:
                parser.error("baudrate must be a number, not %r" % args[0])
            args.pop(0)
        if args:
            parser.error("too many arguments")
    else:
        if port is None:
            port = 0
    if options.cr and options.lf:
        parser.error("ony one of --cr or --lf can be specified")
    if options.filename or options.freq:
        if (not (options.filename)) or (not (options.freq)):
            parser.error("--file and --freq must always be used together")
        else:
            if options.freq <= 0.0:
                parser.error("Freq must be > 0 Hz")

            if not os.path.exists(options.filename):
                parser.error("File '%s' does not exist" % options.filename)


def init_serial_interface():
    try:
        ser.open()
    except serial.SerialException, e:
        print "Could not open serial port %s: %s" % (ser.portstr, e)
        sys.exit(1)
    if options.rts_state is not None:
        ser.setRTS(options.rts_state)
    if options.dtr_state is not None:
        ser.setDTR(options.dtr_state)


if __name__ == '__main__':

    initialize_options()

    port = options.port
    baudrate = options.baudrate

    assert_options_are_valid()

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.rtscts = options.rtscts
    ser.xonxoff = options.xonxoff
    ser.timeout = 1  # required so that the reader thread can exit

    if not options.quiet:
        print "--- RS485 MESSAGE HANDLER --- type Ctrl-C / BREAK to quit"
        print "--- %s %s,%s,%s,%s ---" % (
            ser.portstr, ser.baudrate, 8, ser.parity, 1)
        if options.filename:
            print "--- reading from %s at %.2f Hz ---" % (
                options.filename, options.freq)

    # Initting ROS node
    rospy.init_node('interface_rs485', anonymous=True)
    while not rospy.is_shutdown():
        try:
            init_serial_interface()
            r = RS485MessageHandler(ser,options.sniff)
            time.sleep(1)
            r.initializer() # this is blocking
            ser.close()
            print('Port closed', ser)
        except rospy.ROSInterruptException:
            break

    print("\n--- exit ---")
