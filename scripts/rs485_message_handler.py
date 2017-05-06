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
import serial
import socket
import sys
import threading
import time
import struct
import binascii

import rospy
from Queue import Queue
from optparse import (OptionParser)
from interface_rs485.msg import SendRS485Msg

class RS485Msg:
    def __init__(self, msg):
        self.start = 0x3A
        self.slave = msg.slave
        self.command = msg.cmd  # on envoie la commande qui demande l'etat de la switch mission
        self.nbByte = len(msg.data)
        self.data = list(struct.unpack("{}B".format(self.nbByte), msg.data))
        self.end = 0x0D
        self.checksum = self.start + self.slave + self.command + self.nbByte
        for d in self.data:
            self.checksum += d
        self.checksum += self.end


class RS485MessageHandler:

    def __init__(self, serial_name, sniff):
        self.serial = serial_name
        self.socket = socket
        self.sniff = sniff
        self.count_read_data = 0
        self.max_read_data_print = 200
        self.count_write_data = 0
        self.max_write_data_print = 200
        self.queue_to_write = Queue()
        self.int_lst = list()

        self.pub = rospy.Publisher('/interface_rs485/dataTx', SendRS485Msg, queue_size=10)
        self.sub = rospy.Subscriber('/interface_rs485/dataRx', SendRS485Msg, self.receive_from_subscriber)
        # rospy.Publisher('/tobechange', MissionSwitchMsg, queue_size=10)

    def initializer(self):
        self.thread_read = threading.Thread(target=self.reader)
        self.thread_read.setDaemon(1)
        self.thread_read.start()
        #self.thread_test = threading.Thread(target=self.cmd_line_test)
        #self.thread_test.start()

        # sub = rospy.Subscriber('/interface_rs485/SendRS485Msg', SendRS485Msg, self.sendRS485_message_received()
        # sub = rospy.Subscriber('/interface_rs485/SendRS485Msg', bytes, queue_size=100)
        self.thread_write = threading.Thread(target=self.writer)
        self.thread_write.setDaemon(1)
        self.thread_write.start()
        rospy.spin()

        # sub.unregister()

    def cmd_line_test(self):
        while 1:
            raw_input("press enter\n")
            sendRs485Msg = SendRS485Msg()
            sendRs485Msg.slave = 0x20
            sendRs485Msg.cmd = 0x00
            sendRs485Msg.data = [0x00]
            sendRs485Msg1 = SendRS485Msg()
            sendRs485Msg1.slave = 0x20
            sendRs485Msg1.cmd = 0x01
            sendRs485Msg1.data = [0x00]
            msg = RS485Msg(sendRs485Msg)
            msg1 = RS485Msg(sendRs485Msg1)
            self.queue_to_write.put(msg)
            self.queue_to_write.put(msg1)

    def receive_from_subscriber(self, send_rs485_msg):
        msg = RS485Msg(send_rs485_msg)
        self.queue_to_write.put(msg)

    def reader(self):
        """loop forever and copy serial->socket"""
        while not rospy.is_shutdown():
            try:

                data = self.serial.read(1)  # read one, blocking
                # print ord(data)
                n = self.serial.inWaiting()  # look if there is more
                if n:
                    data = data + self.serial.read(n)  # and get as much as possible

                if data:
                    # Pipe to stdout if sniffing
                    if self.sniff:
                        print('Read:', data)
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
                send_rs485_msg = self.queue_to_write.get()  # this is blocking.
                if send_rs485_msg:
                    # Pipe to stdout if sniffing
                    if True:  # self.sniff:
                        # print('Write:', send_rs485_msg)
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

    def send_rs485_message_received(self, send_rs485_msg):
        self.queue_to_write.put(send_rs485_msg)

    def write_data_to_rs485(self, send_rs485_msg):
        data = list()

        data.append(send_rs485_msg.start)
        data.append(send_rs485_msg.slave)
        data.append(send_rs485_msg.command)
        data.append(send_rs485_msg.nbByte)
        checksum = 0
        for data_byte in send_rs485_msg.data:
            data.append(data_byte)
            checksum += data_byte
        checksum += send_rs485_msg.start + send_rs485_msg.slave + send_rs485_msg.command + send_rs485_msg.nbByte\
            + send_rs485_msg.end
        data.append((checksum >> 8) & 0xFF)
        data.append(checksum & 0xFF)
        data.append(send_rs485_msg.end)

        self.serial.write(data)
        # gen = random.Random()
        # if gen.random() > 0.5:
        #     junk = gen.randint(0, 255)
        #     print "writing junk: {}".format(junk)
        #     self.serial.write([junk])
        # raw_input(data)
        pass

    def new_data_read_from_rs485(self, data):


        # data is bytes. accessible via data[0], data[1]
        # on regarde l'integrite du message si il n'est pas bon on pop un byte
        for x in data:
            self.int_lst.append(ord(x))
        if len(self.int_lst) >= 8:
            while len(self.int_lst) > 0 and self.int_lst[0] != 0x3A:
                self.int_lst.pop(0)
            if len(self.int_lst) >= 8:
                # print self.int_lst
                receive_rs485_msg = RS485Msg(SendRS485Msg())
                receive_rs485_msg.start = self.int_lst[0]
                receive_rs485_msg.slave = self.int_lst[1]
                receive_rs485_msg.command = self.int_lst[2]
                receive_rs485_msg.nbByte = self.int_lst[3]
                # print "banana"
                if len(self.int_lst) >= 7 + receive_rs485_msg.nbByte:
                    k = 0
                    checksum_calc = 0x3A + receive_rs485_msg.slave + receive_rs485_msg.command +\
                        receive_rs485_msg.nbByte + 0x0D
                    while k < receive_rs485_msg.nbByte:
                        receive_rs485_msg.data.append(self.int_lst[4+k])
                        checksum_calc += self.int_lst[4+k]
                        k += 1
                    checksum_calc &= 0xFFFF

                    receive_rs485_msg.checksum = self.int_lst[4+k] << 8 | self.int_lst[5+k]
                    receive_rs485_msg.end = self.int_lst[6+k]
                    # print self.int_lst[4 + k]
                    # print self.int_lst[5 + k]
                    # print receive_rs485_msg.checksum
                    # print checksum_calc
                    # print "banana2"
                    if receive_rs485_msg.start == 0x3A and receive_rs485_msg.end == 0x0D and\
                            receive_rs485_msg.checksum == checksum_calc:
                        # print "banana3"
                        #
                        # print self.int_lst
                        for i in range(1, receive_rs485_msg.nbByte + 7):
                            self.int_lst.pop(0)
                        # ma_mission = MissionSwitchMsg()
                        # ma_mission.state = self.receive_rs485_msg.data[0]
                        mon_msg = SendRS485Msg()
                        mon_msg.slave = receive_rs485_msg.slave
                        mon_msg.cmd = receive_rs485_msg.command
                        mon_msg.data = receive_rs485_msg.data
                        # print "slave %d , cmd %d , data %d." % (mon_msg.slave, mon_msg.cmd, receive_rs485_msg.data[0])
                        self.pub.publish(mon_msg)  # on envoie le message a tous les subscriber
                        # if receive_rs485_msg.nbByte == 4:
                            # v1 = chr(receive_rs485_msg.data[0])
                            # v2 = chr(receive_rs485_msg.data[1])
                            # v3 = chr(receive_rs485_msg.data[2])
                            # v4 = chr(receive_rs485_msg.data[3])
                            # packed_data = v1+v2+v3+v4
                            # s = struct.Struct('f')
                            # data123 = s.unpack(packed_data)
                            ## print "slave %d , cmd %d , data %f." % (mon_msg.slave, mon_msg.cmd, data123[0])
                            # pour tester dans la console
                            #
                            #if receive_rs485_msg.command == 0x06:
                            #    print('{0:d} , {1:d}'.format(receive_rs485_msg.data[0], receive_rs485_msg.data[1]))
                            #if receive_rs485_msg.command == 0x00:
                            #    print("mission = %d" % receive_rs485_msg.data[0])
                    else:
                        self.int_lst.pop(0)

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
                      default="/dev/ttyS4")
    parser.add_option("-b", "--baud", dest="baudrate", action="store",
                      type='int',
                      help="set baudrate, default 115200", default=115200)
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
    # if options.filename or options.freq:
    #    if (not (options.filename)) or (not (options.freq)):
    #        parser.error("--file and --freq must always be used together")
    #    else:
    #        if options.freq <= 0.0:
    #            parser.error("Freq must be > 0 Hz")

    #        if not os.path.exists(options.filename):
    #            parser.error("File '%s' does not exist" % options.filename)


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
        print "--- %s %s,%s,%s,%s ---" % (ser.portstr, ser.baudrate, 8, ser.parity, 1)
        # if options.filename:
        #     print "--- reading from %s at %.2f Hz ---" % (options.filename, options.freq)

    # Initting ROS node
    rospy.init_node('interface_rs485', anonymous=True)
    while not rospy.is_shutdown():
        try:
            init_serial_interface()
            r = RS485MessageHandler(ser, options.sniff)
            time.sleep(1)
            r.initializer()  # this is blocking
            ser.close()
            print('Port closed', ser)
        except rospy.ROSInterruptException:
            break

    print("\n--- exit ---")
