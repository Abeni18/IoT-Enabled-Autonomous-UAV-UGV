#!/usr/bin/env python
# Author: Mrinmoy sarkar
# email: msarkar@aggies.ncat.edu


import rospy
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import TransformStamped
from scipy.spatial import distance
from copy import deepcopy
import pandas as pd
from matplotlib import pyplot as plt
from sklearn.cluster import KMeans
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import matplotlib.pyplot as plt


X = np.zeros((3,2))

drone1SetPointPub = None
drone2SetPointPub = None
global_x_pub = None
global_y_pub = None
fig = plt.gcf()
fig.show()
fig.canvas.draw()

dron1x = 0
dron1y = 0
dron2x = 0
dron2y = 0


def car1_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[0,0] = x 
    X[0,1] = y 


def car2_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[1,0] = x 
    X[1,1] = y 

def car3_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[2,0] = x 
    X[2,1] = y
   

def car4_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[3,0] = x 
    X[3,1] = y
   

def car5_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[4,0] = x 
    X[4,1] = y


def car6_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[5,0] = x 
    X[5,1] = y
   

def car7_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[6,0] = x 
    X[6,1] = y
   

def car8_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[7,0] = x 
    X[7,1] = y
   

def car9_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[8,0] = x 
    X[8,1] = y


def car10_cb(msg):
    global X 
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    X[9,0] = x 
    X[9,1] = y


def quad1_cb(msg):
    global dron1x
    global dron1y
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    dron1x = x 
    dron1y = y

def custom2_cb(msg):
    global dron2x
    global dron2y
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    dron2x = x 
    dron2y = y


def calculateCluster():
    global X 
    global drone1SetPointPub
    global drone2SetPointPub
    global global_x_pub
    global global_y_pub
    global fig
    global dron1x
    global dron1y
    global dron2x
    global dron2y

    kmeans  = KMeans(n_clusters=2, random_state=0).fit(X)
    centers = kmeans.cluster_centers_
    labels  = kmeans.labels_
    # print(centers)
    # print("*******")
    # drone1Center = Float32MultiArray()
    # drone2Center = Float32MultiArray()

    # drone1Center.data = centers[0]
    # drone2Center.data = centers[1]

    # drone1SetPointPub.publish(drone1Center)
    # drone2SetPointPub.publish(drone2Center)
    # global_x_pub.publish(centers[0][0])
    # global_y_pub.publish(centers[0][1])

    fig.clear()
    if centers[0][1]>centers[1][1]:
        plt.scatter(centers[0][0],centers[0][1],marker='h',label="cluster_center1",color='r',linewidth=5)
        plt.scatter(centers[1][0],centers[1][1],marker='h',label="cluster_center2",color='g',linewidth=5)
    else:
        plt.scatter(centers[0][0],centers[0][1],marker='h',label="cluster_center2",color='r',linewidth=5)
        plt.scatter(centers[1][0],centers[1][1],marker='h',label="cluster_center1",color='g',linewidth=5)
    plt.scatter(X[0][0],X[0][1],label="car1",color='b')
    plt.scatter(X[1][0],X[1][1],label="car2",color='b')
    plt.scatter(X[2][0],X[2][1],label="car3",color='b')
    # plt.scatter(X[3][0],X[3][1],label="car4",color='b')
    # plt.scatter(X[4][0],X[4][1],label="car5",color='b')
    # plt.scatter(X[5][0],X[5][1],label="car6",color='b')
    # plt.scatter(X[6][0],X[6][1],label="car7",color='b')
    # plt.scatter(X[7][0],X[7][1],label="car8",color='b')
    plt.scatter(dron1x,dron1y,marker='D',label="UAV1",color='k',linewidth=5)
    plt.scatter(dron2x,dron2y,marker='D',label="UAV2",color='m',linewidth=5)
    

    plt.xlim([-1,3])
    plt.ylim([-3,7])
    plt.legend()
    plt.pause(0.0001)
    fig.canvas.draw()



if __name__ == '__main__':
    rospy.init_node('car_data_read', anonymous=True)
    # drone1SetPointPub = rospy.Publisher("/drone1SetPoint",Float32MultiArray,queue_size=100)
    # drone2SetPointPub = rospy.Publisher("/drone2SetPoint",Float32MultiArray,queue_size=100)
    # global_x_pub = rospy.Publisher("/global_x",Float32,queue_size=100)
    # global_y_pub = rospy.Publisher("/global_y",Float32,queue_size=100)
    rospy.Subscriber("/vicon/car1/car1", TransformStamped, car1_cb)
    rospy.Subscriber("/vicon/car2/car2", TransformStamped, car2_cb)
    rospy.Subscriber("/vicon/car3/car3", TransformStamped, car3_cb)
    # rospy.Subscriber("/vicon/car4/car4", TransformStamped, car4_cb)
    # rospy.Subscriber("/vicon/car5/car5", TransformStamped, car5_cb)
    # rospy.Subscriber("/vicon/car6/car6", TransformStamped, car6_cb)
    # rospy.Subscriber("/vicon/car7/car7", TransformStamped, car7_cb)
    # rospy.Subscriber("/vicon/car8/car8", TransformStamped, car8_cb)
    # rospy.Subscriber("/vicon/car9/car9", TransformStamped, car9_cb)
    # rospy.Subscriber("/vicon/car10/car10", TransformStamped, car10_cb)
    # rospy.Subscriber("/vicon/car5/car5", TransformStamped, car5_cb)
    # rospy.Subscriber("/vicon/car6/car6", TransformStamped, car6_cb)
    rospy.Subscriber("/vicon/quad1/quad1", TransformStamped, quad1_cb)
    rospy.Subscriber("/custom2/mavros/mocap/tf", TransformStamped, custom2_cb)
    

    while not rospy.is_shutdown():
        rospy.sleep(1)
        calculateCluster()