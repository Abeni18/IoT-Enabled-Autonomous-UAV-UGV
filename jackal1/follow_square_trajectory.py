#!/usr/bin/env python
# Author: Mrinmoy sarkar
# email: msarkar@aggies.ncat.edu
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
#import pandas as pd
import threading
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, atan2, sqrt


stopThread = False
pub = None


X = np.zeros((3,2))
x_max = 1.73
x_min = 0.15
y_max = 9.5
y_min = 0
x_data = []
y_data = []

drone1SetPointPub = None
drone2SetPointPub = None
yaw = 0

def jackle2_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[1,0] = x 
    X[1,1] = y 


def jackle1_cb(msg):
    global X 
    global yaw, x_data,y_data
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    
    X[0,0] = x 
    X[0,1] = y
    x_data.append(x)
    y_data.append(y)
    orientation_list = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    yaw = degrees(yaw)
    # print(yaw)
    

def move(pub,x,y,z):
    twist = Twist()
    speed = 0.3  
    twist.linear.x = x*speed 
    twist.linear.y = y*speed
    twist.linear.z = z*speed
    twist.angular.x = 0 
    twist.angular.y = 0 
    twist.angular.z = 0
    pub.publish(twist)


def rotate(pub,deg,leftorright):
    global yaw, stopThread
    twist = Twist()
    speed = 0.3  
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0 
    if leftorright:
        twist.angular.z = speed
    else:
        twist.angular.z = -speed
    last_yaw = yaw
    rate = rospy.Rate(10.0)
    move(pub,0,0,0)
    while abs(yaw-last_yaw) < deg and not stopThread:
        pub.publish(twist)
        rate.sleep()
    move(pub,0,0,0) 
    

def rotateZero(pub):
    global yaw, stopThread
    rate = rospy.Rate(10.0)
    twist = Twist()
    speed = 0.3  
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0 
    if yaw>0:
        twist.angular.z = -speed
    else:
        twist.angular.z = speed
    
    
    move(pub,0,0,0)
    while abs(yaw) > 2 and not stopThread:
        pub.publish(twist)
        rate.sleep()
    move(pub,0,0,0)

def rotateabsoluteAngle(pub,ang):
    global yaw, stopThread
    rate = rospy.Rate(10.0)
    twist = Twist()
    speed = 0.3  
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0 

    dl = ang - yaw
    # print("dl:",dl,"ang:",ang,"yaw",yaw)

    if (ang >=0 and yaw >=0) or (ang < 0 and yaw < 0):
        if dl >= 0: 
            twist.angular.z = speed #ccw
        else:
            twist.angular.z = -speed #cw
    elif (ang >=0 and yaw <=0):
        if dl <= 180: 
            twist.angular.z = speed #CCW
        else:
            twist.angular.z = -speed #CW
    elif (ang <= 0 and yaw >=0):
        if dl <= -180: 
            twist.angular.z = speed
        else:
            twist.angular.z = -speed
    
    
    move(pub,0,0,0)
    # print(ang)
    # print(yaw)
    while abs(ang-yaw) > 2 and not stopThread:
        pub.publish(twist)
        rate.sleep()
    move(pub,0,0,0) 

def goto_2dLocation(pub,x,y):
    global X, yaw, stopThread
    #calculate desired angle
    dy = y-X[0,1]
    dx = x-X[0,0]
    dang = degrees(atan2(dy,dx))
    rotateabsoluteAngle(pub,dang)
    d = sqrt(dy**2+dx**2)

    rate = rospy.Rate(10.0)
    twist = Twist()
    speed = 0.3  
    twist.linear.x = 0.3
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0 
    twist.angular.y = 0
    twist.angular.z = 0
    kp_ang = 0.05
    kp_dis = 0.1
    counter = 0
    while d > 0.15 and counter<100 and not stopThread:
        dy = y-X[0,1]
        dx = x-X[0,0]
        d = sqrt(dy**2+dx**2)
        twist.linear.x = max(min(kp_dis*d, 0.7),0.2)
        twist.angular.z = min(kp_ang*(dang-yaw), 0.3)
        pub.publish(twist)
        # print(twist.angular.z)
        # print("d:",d)
        rate.sleep()
        counter += 1
    move(pub,0,0,0)
    

def loop():
    global stopThread
    global pub
    global X
    global x_max
    global x_min
    global y_max, y_min
    start_x = X[0,0]
    stary_y = X[0,1]

    rate = rospy.Rate(10.0)
    rotateZero(pub)
    rospy.sleep(1)
    goto_2dLocation(pub,1.8,1.86)
    rospy.sleep(1)
    goto_2dLocation(pub,1.8,4.0)
    rospy.sleep(1)
    goto_2dLocation(pub,0,4.0)
    rospy.sleep(1)
    goto_2dLocation(pub,0,1.86)
    rospy.sleep(1)
    rotateZero(pub)
    rospy.sleep(1)
    goto_2dLocation(pub,0.7,1.86)
    rospy.sleep(1)
    # while not rospy.is_shutdown() and not stopThread:
    #     while X[0,0] < x_max:
    #         move(pub,1,0,0)
    #         rate.sleep()
    #     move(pub,0,0,0)
    #     rospy.sleep(1)

    #     rotateabsoluteAngle(pub,90)
    #     rospy.sleep(1)

    #     start_x = X[0,0]
    #     stary_y = X[0,1]
    #     while abs(X[0,1]-stary_y) < 2:
    #         move(pub,1,0,0)
    #         rate.sleep()
    #     move(pub,0,0,0)
    #     rospy.sleep(1)

    #     rotateabsoluteAngle(pub,180)
    #     rospy.sleep(1)
        
    #     while X[0,0] > x_min:
    #         move(pub,1,0,0)
    #         rate.sleep()
    #     move(pub,0,0,0)
    #     rospy.sleep(1)

    #     rotateabsoluteAngle(pub,-90)
    #     rospy.sleep(1)

    #     start_x = X[0,0]
    #     stary_y = X[0,1]

    #     while abs(X[0,1]-stary_y) < 2:
    #         move(pub,1,0,0)
    #         rate.sleep()
    #     move(pub,0,0,0)
    #     rospy.sleep(1)

    #     rotateabsoluteAngle(pub,0)
    #     rospy.sleep(1)

    #     rospy.sleep(10)

    move(pub,0,0,0)    
    print("stopping the loop")


if __name__ == '__main__':          
    try:
        rospy.init_node('move_jackle_node', anonymous=True)
        rospy.Subscriber("/vicon/jackal1/jackal1", TransformStamped, jackle1_cb)
        rospy.Subscriber("/vicon/jackal2/jackal2", TransformStamped, jackle2_cb)

        pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  
        rospy.sleep(5)
        # rotateZero(pub)
        # rotateabsoluteAngle(pub,-50)

        threading.Thread(target=loop).start()

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            rate.sleep()
            cmd = raw_input("command: \"s\" to stop the UGV and \"exit\" for exit the main loop: ")
            # print(cmd)
            if cmd=='s':
                stopThread = True
            elif cmd=='exit':
                stopThread = True
                rospy.sleep(5)
                break

    except rospy.ROSInterruptException:
        pass

