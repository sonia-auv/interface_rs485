#!/usr/bin/env python3
import rospy
import numpy
import struct
import os

from sonia_common.msg import *


def sim_power_auv7():
    rospy.init_node('interface_rs485_simulation', anonymous=True)
    pub = rospy.Publisher('/interface_rs485/dataTx', SendRS485Msg, queue_size=100)
    rate = rospy.Rate(1)  # 10hz
    msg = SendRS485Msg()
    while not rospy.is_shutdown():
        for i in range(0, 4):
            msg.slave = i
            msg.cmd = 0
            msg.data = voltage_generation_AUV7()
            pub.publish(msg)

            msg.cmd = 1
            msg.data = current_generation(3)
            pub.publish(msg)

            msg.cmd = 15
            msg.data = motor_generation(2)
            pub.publish(msg)

        rate.sleep()


def voltage_generation_AUV7():
    rand = []
    for j in range(0,3):
        rand.append(numpy.random.uniform(14.4, 16.8))
    rand.append(numpy.random.uniform(11.5,13))

    return transform(rand)

def voltage_generation_AUV8():
    rand = []
    for j in range (0,10):
        rand.append(numpy.random.uniform(14.4, 16.8))
    return transform(rand)

def current_generation(nb):
    rand = []
    for j in range(0,nb):
        rand.append(numpy.random.uniform(0, 25))

    return transform(rand)

def motor_generation(nb):
    rand = []
    for j in range(0,nb):
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


def sim_power_auv8():
    rospy.init_node('interface_rs485_simulation', anonymous=True)
    pub = rospy.Publisher('/interface_rs485/dataTx', SendRS485Msg, queue_size=100)
    rate = rospy.Rate(1)  # 10hz
    msg = SendRS485Msg()
    while not rospy.is_shutdown():
        msg.slave = 8
        msg.cmd = 0
        msg.data = voltage_generation_AUV8()
        pub.publish(msg)

        msg.cmd = 1
        msg.data = current_generation(10)
        pub.publish(msg)

        msg.cmd = 15
        msg.data = motor_generation(8)
        pub.publish(msg)

        rate.sleep()



if __name__ == '__main__':
    try:
        auv = os.getenv('LOCAL_AUV',"AUV7")
        print(auv)
        if (auv=="AUV7"):
            sim_power_auv7()
        else:
            sim_power_auv8()
    except rospy.ROSInterruptException:
        pass

