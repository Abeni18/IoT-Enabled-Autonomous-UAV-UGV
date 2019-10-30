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



X = np.zeros((3,2))

drone1SetPointPub = None
drone2SetPointPub = None


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


def calculateCluster():
    global X 
    global drone1SetPointPub
    global drone2SetPointPub
    kmeans  = KMeans(n_clusters=2, random_state=0).fit(X)
    centers = kmeans.cluster_centers_
    labels  = kmeans.labels_
    print(centers)
    print("*******")
    drone1Center = Float32MultiArray()
    drone2Center = Float32MultiArray()

    if centers[0][1]>centers[1][1]:
        drone1Center.data = centers[0]
        drone2Center.data = centers[1]
    else:
        drone1Center.data = centers[1]
        drone2Center.data = centers[0]

    drone1SetPointPub.publish(drone1Center)
    drone2SetPointPub.publish(drone2Center)

if __name__ == '__main__':
    rospy.init_node('car_data_read', anonymous=True)
    drone1SetPointPub = rospy.Publisher("/drone1SetPoint",Float32MultiArray,queue_size=100)
    drone2SetPointPub = rospy.Publisher("/drone2SetPoint",Float32MultiArray,queue_size=100)
    rospy.Subscriber("/vicon/car1/car1", TransformStamped, car1_cb)
    rospy.Subscriber("/vicon/car2/car2", TransformStamped, car2_cb)
    rospy.Subscriber("/vicon/car3/car3", TransformStamped, car3_cb)
    rospy.Subscriber("/vicon/car4/car4", TransformStamped, car4_cb)
    rospy.Subscriber("/vicon/car5/car5", TransformStamped, car5_cb)
    rospy.Subscriber("/vicon/car6/car6", TransformStamped, car6_cb)
    rospy.Subscriber("/vicon/car7/car7", TransformStamped, car7_cb)
    rospy.Subscriber("/vicon/car8/car8", TransformStamped, car8_cb)
    rospy.Subscriber("/vicon/car9/car9", TransformStamped, car9_cb)
    rospy.Subscriber("/vicon/car10/car10", TransformStamped, car10_cb)

    while not rospy.is_shutdown():
        rospy.sleep(3)
        calculateCluster()