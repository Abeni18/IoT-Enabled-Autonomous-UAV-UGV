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
import serial
from time import sleep


stopThread = False
startStop = False
pub = None


X = np.zeros((3,2))
x_max = 1.73
x_min = 0.15
y_max = 9.5
y_min = 0
x_data = []
y_data = []
delx = 0
dely = 0

grid = []

drone1SetPointPub = None
drone2SetPointPub = None
yaw = 0

class point2d:
    def __init__(self,x,y):
        self.x=x
        self.y=y

def initGrid():
    global grid, x_min, x_max, y_min, y_max, delx, dely

    x_grid = np.linspace(x_min,x_max,3)
    y_grid = np.linspace(y_min,y_max,10)
    delx = x_grid[1] - x_grid[0]
    dely = y_grid[1] - y_grid[0]

    for i in range(len(x_grid)):
        for j in range(len(y_grid)):
            point = point2d(x_grid[i],y_grid[j])
            grid.append(point)

def printGrid():
    global grid
    for i in range(len(grid)):
        print(grid[i].x,grid[i].y) 

def jackle1_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[0,0] = x 
    X[0,1] = y 

def jackle2_cb(msg):
    global X 
    global yaw, x_data,y_data
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[1,0] = x 
    X[1,1] = y 
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
    global yaw
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
    while abs(yaw-last_yaw) < deg:
        pub.publish(twist)
        rate.sleep()
    move(pub,0,0,0) 
    
def rotateZero(pub):
    global yaw
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
    while abs(yaw) > 2:
        pub.publish(twist)
        rate.sleep()
    move(pub,0,0,0)

def rotateabsoluteAngle(pub,ang):
    global yaw
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
    while abs(ang-yaw) > 2:
        pub.publish(twist)
        rate.sleep()
    move(pub,0,0,0) 
    
def goto_2dLocation(pub,x,y):
    global X, yaw
    #calculate desired angle
    dy = y-X[1,1]
    dx = x-X[1,0]
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
    while d > 0.15 and counter<100:
        dy = y-X[1,1]
        dx = x-X[1,0]
        d = sqrt(dy**2+dx**2)
        twist.linear.x = max(min(kp_dis*d, 0.7),0.2)
        twist.angular.z = min(kp_ang*(dang-yaw), 0.3)
        pub.publish(twist)
        # print(twist.angular.z)
        # print("d:",d)
        rate.sleep()
        counter += 1
    move(pub,0,0,0)

def getNext2dLocation():
    global X, grid, delx, dely, x_min, x_max, y_min, y_max
    min_x = 0
    min_y = 0 
    mind = 10e10
    for i in range(len(grid)):
        dx = grid[i].x - X[1,0]
        dy = grid[i].y - X[1,1]
        d = (dx**2+dy**2)
        if d <= mind:
            mind = d
            min_x = grid[i].x
            min_y = grid[i].y
    allxs = []
    allys = []
    for i in [0,1,-1]:
        for j in [0,1,-1]:
            if i==0 and j==0:
                continue
            else:
                x = min_x + i*delx
                y = min_y + j*dely

                if x >= x_min and x <= x_max and y >= y_min and y <= y_max:
                    allxs.append(x)
                    allys.append(y)
    indx = np.random.randint(len(allxs))
    x = allxs[indx]
    y = allys[indx]
    # print(x,y)
    return x,y

def checkCollision():
    global X, y_max, y_min, pub
    dx = X[0,0]-X[1,0]
    dy = X[0,1]-X[1,1]
    d= sqrt(dx**2+dy**2)
    if d < 2.0:
        if X[1,1] > X[0,1]:
            goto_2dLocation(pub, X[1,0], X[1,1]+(y_max - X[1,1])/2)
        else:
            goto_2dLocation(pub, X[1,0], X[1,1]/2)

def loop():
    global stopThread, X, pub, startStop
    
    while not rospy.is_shutdown() and not stopThread:
        if startStop:
            checkCollision()
            x,y = getNext2dLocation()
            goto_2dLocation(pub,x,y)
        rospy.sleep(1)

    move(pub,0,0,0)    
    print("stopping the loop")


def readserial():
    global startStop, stopThread
    port = "/dev/ttyUSB0"
    ser = serial.Serial(port, 115200, timeout=0)

    while not stopThread:
        data = ser.read(9999)
        #'Message arrived on topic: UAV_1/UGV_4/Control. Message: 5\r\ncommand output 5\r\n\r\n')

        if len(data) > 0:
            #print('Got:', data)
            indx = data.find('command output ')
            if indx != -1:
                indx += len('command output ') 
                if data[indx]=="8":
                    print("start")
                    startStop = True
                elif data[indx]=="5":
                    print("stop")
                    startStop = False
                elif data[indx]=="4":
                    print("left")
                elif data[indx]=="6":
                    print("right")
                elif data[indx]=="2":
                    print("back")

        sleep(0.5)
        #print('not blocked')

    ser.close()
    print("serial close")



if __name__ == '__main__':          
    try:
        rospy.init_node('move_jackle_node', anonymous=True)
        rospy.Subscriber("/vicon/jackal1/jackal1", TransformStamped, jackle1_cb)
        rospy.Subscriber("/vicon/jackal2/jackal2", TransformStamped, jackle2_cb)

        pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
        initGrid()
        # printGrid()
        rospy.sleep(5)
        # print("*********")
        # for i in range(4):
        #     getNext2dLocation()
        
        
        # goto_2dLocation(pub,1,0)
        # rospy.sleep(5)
        # rotateZero(pub)
        # rotateabsoluteAngle(pub,-50)

        threading.Thread(target=loop).start()
        threading.Thread(target=readserial).start()

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            rate.sleep()
            cmd = raw_input("command: \"s\" to stop the UGV and \"exit\" for exit the main loop: ")
            if cmd=='s':
                stopThread = True
            elif cmd=='exit':
                break

    except rospy.ROSInterruptException:
        pass

