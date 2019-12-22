#!/usr/bin/env python
# -*- coding: utf-8 -*-
import serial
import time
import rospy
import pyaudio
import wave
a=0
CHUNK = 1024
filename="sinwave.wav"

from std_msgs.msg import Int32
ser = serial.Serial('/dev/ttyACM0', 9600)

def callback(msg):
    rospy.loginfo(msg.data)
    if (msg.data==1):
        ser.write('s')
    elif (msg.data==0):
        ser.write('q')
        

def callback2(msg):
    rospy.loginfo(msg.data)
    if (msg.data==1):
        a=1
    elif (msg.data==0):
        a=0

def listener():
    rospy.init_node('listener')
    rospy.Subscriber('magnetflag',Int32,callback)
    rospy.spin()

def listener2():
    rospy.init_node('listener2')
    rospy.Subscriber('playsound',Int32,callback2)
    rospy.spin()

wf = wave.open(filename, 'rb')

p = pyaudio.PyAudio()

stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True)

print("begin")
data = wf.readframes(CHUNK)
while (a== 1 and data != ''):
    stream.write(data)

listener()
listener2()
print("end")
ser.close()
