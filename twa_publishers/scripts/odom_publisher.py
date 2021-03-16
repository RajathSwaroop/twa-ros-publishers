#!/usr/bin/env python
import csv

import math
import time
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('twa_odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

vx = 0.1
vy = -0.1
vth = 0.1

current_time = rospy.Time.now()
last_time = rospy.Time.now()

idx = 0
odom_data = []
with open('/home/rajath/logs/odom_20210224.csv') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in spamreader:
        odom_data.append( row )

sleep_duration = 1.0/48
prev_x = 0
prev_y = 0
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    #current_time.secs = int( odom_data[idx][1] )
    #current_time.nsecs = int( odom_data[idx][3] )
    '''
    if idx + 1 < len(odom_data):
        next_time = rospy.Time.now()
        next_time.secs = int(odom_data[idx+1][1])
        next_time.nsecs = int(odom_data[idx+1][3])
        sleep_duration = ( next_time - current_time ).nsecs / pow(10.0,9)

    print(sleep_duration)
    '''
    # compute odometry in a typical way given the velocities of the robot
    x = float(odom_data[idx][5])
    y = float(odom_data[idx][7])

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = [ float(odom_data[idx][9]), float(odom_data[idx][11]), float(odom_data[idx][13]), float(odom_data[idx][15]) ]#tf.transformations.quaternion_from_euler(0, 0, th)
    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(0., 0., 0.), Quaternion(*odom_quat))
    odom.pose.covariance[0] = 0.1
    odom.pose.covariance[7] = 0.1
    odom.pose.covariance[35] = 0.1

    vx = ( x - prev_x ) / sleep_duration
    vy = ( y - prev_y ) / sleep_duration
    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, 0))
    odom.twist.covariance[0] = 0.001
    odom.twist.covariance[7] = 0.001
    odom.twist.covariance[35] = 0.1

    # publish the message
    odom_pub.publish(odom)
    prev_x = x
    prev_y = y
    time.sleep(sleep_duration)
    idx += 1
