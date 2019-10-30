#!/usr/bin/env python
# Author: Mrinmoy sarkar
# email: msarkar@aggies.ncat.edu

from __future__ import print_function

import rospy
from std_msgs.msg import String
import numpy as np
import cv2

import time
import os
import sys
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, CommandTOL
from math import radians, degrees, pi, cos, sin
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import time
import threading


current_state = None
current_alt = 0
hx=0.9
hy=1.6
hz=2.0
curx = 0
cury = 0
setpointPub = None 
stopThread = False
setpoint = None
x,y=0.93,1.67


def state_cb(msg):
    global current_state
    current_state = msg
    #print(current_state)

def set_arm(arm, timeout):
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    rospy.wait_for_service('/custom2/mavros/cmd/arming')
    arming_srv = rospy.ServiceProxy('/custom2/mavros/cmd/arming', CommandBool)
    for i in xrange(timeout * loop_freq):
        if current_state.armed == arm:
            return True
        else:
            try:
                res = arming_srv(arm)
                if not res.success:
                    rospy.logerr("failed to send arm command")
                else:
                    return True
            except rospy.ServiceException as e:
                rospy.logerr(e)
        try:
            rate.sleep()
        except rospy.ROSException as e:
            rospy.logerr(e)
    return False            

def land(timeout):
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    rospy.wait_for_service('/custom2/mavros/cmd/land')
    land_set = rospy.ServiceProxy('/custom2/mavros/cmd/land', CommandTOL)
    
    for i in xrange(timeout * loop_freq):
        if current_state.armed == False:
            return True
        else:
            try:
                res = land_set(min_pitch=0,
                         yaw=0,
                         latitude=0,
                         longitude=0,
                         altitude=0)  # 0 is custom mode
                # res = land_set()
                print(res)
                if not res.success:
                    rospy.logerr("failed to send land command")
                else:
                    return True
            except rospy.ServiceException as e:
                rospy.logerr(e)

        try:
            rate.sleep()
        except rospy.ROSException as e:
            print(e)

    return False
    
def set_mode(mode, timeout):
    loop_freq = 1  # Hz
    rate = rospy.Rate(loop_freq)
    rospy.wait_for_service('/custom2/mavros/set_mode')
    mode_set = rospy.ServiceProxy('/custom2/mavros/set_mode', SetMode)
    for i in xrange(timeout * loop_freq):
        if current_state.mode == mode:
            return True
        else:
            try:
                res = mode_set(0,mode)  # 0 is custom mode
                if not res.mode_sent:
                    rospy.logerr("failed to send mode command")
                else:
                    return True
            except rospy.ServiceException as e:
                rospy.logerr(e)

        try:
            rate.sleep()
        except rospy.ROSException as e:
            self.fail(e)

    return False

def local_position_cb(msg):
    global current_alt
    global curx
    global cury
    current_alt = msg.pose.position.z
    curx = msg.pose.position.x
    cury = msg.pose.position.y

def getquaternion(roll, pitch, yaw):
    # return tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    roll = radians(roll)
    pitch = radians(pitch)
    yaw = radians(yaw)
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    q=[0,0,0,0]
    q[0] = -1*(cy * cp * sr - sy * sp * cr)
    q[1] = -1*(sy * cp * sr + cy * sp * cr)
    q[2] = -1*(sy * cp * cr - cy * sp * sr)
    q[3] = -1*(cy * cp * cr + sy * sp * sr)
    
    return q;

def home_cb(msg):
    global hx
    global hy
    global hz
    hx = msg.transform.translation.x
    hy = msg.transform.translation.y
    hz = msg.transform.translation.z
    hx = min(max(0.2,hx),1.8)
    hy = min(max(-2.0,hy),6.0)

def jackle1_cb(msg):
    global x, y
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    x = min(max(0.05,x),1.75)
    y = min(max(0.5,y),9)

def sendTargetPos():
    global setpointPub, stopThread, setpoint, x, y, curx, cury
    setpoint.pose.position.x = 0.93
    setpoint.pose.position.y = 1.67
    setpoint.pose.position.z = 2.0
    rate = rospy.Rate(20.0)
    counter = 0
    prex,prey=x,y
    new_counter = 0
    while not rospy.is_shutdown():
        d = ((x-curx)**2+(y-cury)**2)**0.5
        d1 = ((x-prex)**2+(y-prey)**2)**0.5
        if d < 0.15 and d1 < 0.1:
            new_counter += 1
        else:
            new_counter = 0
        if counter > 140:
            setpoint.pose.position.x = x
            setpoint.pose.position.y = y
        setpointPub.publish(setpoint)
        prex,prey=x,y
        rate.sleep()
        counter += 1
        # print(new_counter,'\t', d, '\t', d1)
        if new_counter >= 250:
            stopThread = True
        if stopThread:
            try:
                land(5)
            except Exception as e:
                print(e)
            #print('landed not disarm yet')
            rospy.sleep(3)
            try:
                set_arm(False, 5)
            except Exception as e:
                print(e)
            break

    print("closing sendTargetPos thread")

if __name__ == '__main__':
    rospy.init_node('custom2_hover_and_land_scenario', anonymous=True)
    rospy.Subscriber('/custom2/mavros/state', State, state_cb)
    rospy.Subscriber('/custom2/mavros/local_position/pose',PoseStamped,local_position_cb)
    rospy.Subscriber('/vicon/home/home',TransformStamped,home_cb)
    rospy.Subscriber("/vicon/jackal1/jackal1", TransformStamped, jackle1_cb)

    setpointPub = rospy.Publisher('/custom2/mavros/setpoint_position/local',PoseStamped,queue_size=100)
    setpoint = PoseStamped()
    setpoint.pose.position.x = 0.93
    setpoint.pose.position.y = 1.67
    setpoint.pose.position.z = 2.0
    q = getquaternion(0, 0, 90)
    setpoint.pose.orientation.x = q[0]
    setpoint.pose.orientation.y = q[1]
    setpoint.pose.orientation.z = q[2]
    setpoint.pose.orientation.w = q[3]

    rate = rospy.Rate(20.0)
    for i in range(100):
        setpointPub.publish(setpoint);
        rate.sleep()
    if set_mode('OFFBOARD', 5) and set_arm(True, 5):
        threading.Thread(target=sendTargetPos).start()
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            rate.sleep()
            cmd = raw_input("command: \"l\" for land, \"r\" for rotate 90 degrees then \"q\" for exit the main loop: ")
            # print(cmd)
            if cmd=='l':
                stopThread = True
            elif cmd == 'r':
                q = getquaternion(0, 0, 90)
                setpoint.pose.orientation.x = q[0]
                setpoint.pose.orientation.y = q[1]
                setpoint.pose.orientation.z = q[2]
                setpoint.pose.orientation.w = q[3]
            elif cmd=='q':
                set_arm(False, 5)
                break


        