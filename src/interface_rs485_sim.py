#!/usr/bin/env python3
import rospy
import numpy
import struct

from sonia_common.msg import *


def sim_power_auv7():
    rospy.init_node('interface_rs485_simulation', anonymous=True)
    pub = rospy.Publisher('/interface_rs485/dataTx', SendRS485Msg, queue_size=10)
    rate = rospy.Rate(1)  # 10hz
    while not rospy.is_shutdown():
        msg = SendRS485Msg()
        for i in range(0, 4):
            msg.slave = i
            msg.cmd = 0
            msg.data = voltage_generation()
            pub.publish(msg)

            msg.cmd = 1
            msg.data = current_generation()
            pub.publish(msg)

            msg.cmd = 15
            msg.data = motor_generation()
            pub.publish(msg)
        rate.sleep()


def voltage_generation():
    rand = []
    for j in range(0,3):
        rand.append(numpy.random.uniform(14.4, 16.8))
    rand.append(numpy.random.uniform(11.5,13))

    return transform(rand)


def current_generation():
    rand = []
    for j in range(0,3):
        rand.append(numpy.random.uniform(0, 25))

    return transform(rand)

def motor_generation():
    rand = []
    for j in range(0,8):
        rand.append(numpy.random.randint(3))
    return rand


def transform(rand):
    data = []
    for i in range(0,len(rand)):
        intData = int.from_bytes(bytearray(struct.pack("f",rand[i])),byteorder='big')
        binData = format(intData,"b").zfill(32)
        for j in range(0,4):
            data.append(int(binData[j*8:j*8+8],2))

    return data



if __name__ == '__main__':
    try:
        sim_power_auv7()
    except rospy.ROSInterruptException:
        pass

