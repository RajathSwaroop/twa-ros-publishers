#!/usr/bin/env python
import csv

idx = 0
imu_data = []
with open('/home/rajath/logs/imu_logs_20210224_2.csv') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in spamreader:
        imu_data.append( row )

import time
import rospy
import math
import numpy as np
from time import time, sleep
from sensor_msgs.msg import Imu

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]
    
def talker():
    global idx
    pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.init_node('ros_erle_imu', anonymous=True)
    rate = rospy.Rate(2600)

    msg = Imu()
    deg2rad = 0.0174533
    sleep_duration = 1.0 / 2600
    while not rospy.is_shutdown():
       	imu_pkt = imu_data[idx]
        current_time = rospy.Time.now()
        '''
        current_time.secs = int(imu_pkt[1])
        current_time.nsecs = int(imu_pkt[3])
        if idx + 1 < len(imu_data):
            next_time = rospy.Time.now()
            next_time.secs = int(imu_data[idx+1][1])
            next_time.nsecs = int(imu_data[idx+1][3])
            print(current_time.nsecs)#next_time {next_time}")
            print(next_time.nsecs)#next_time {next_time}")
            sleep_duration = ( next_time - current_time ).nsecs / pow(10.0,9)
            print( sleep_duration )
        '''     
	msg.header.stamp = current_time
        msg.header.frame_id = "imu"
        quat = euler_to_quaternion( float(imu_pkt[9]) * deg2rad, float(imu_pkt[5]) * deg2rad, float(imu_pkt[7]) * deg2rad )
	msg.orientation.x = quat[0]
	msg.orientation.y = quat[1]
	msg.orientation.z = quat[2]
	msg.orientation.w = quat[3]
        msg.orientation_covariance[0] = 0.0001
        msg.orientation_covariance[4] = 0.0001
        msg.orientation_covariance[8] = 0.0001
        
	msg.angular_velocity.x = float(imu_pkt[11]) * deg2rad
	msg.angular_velocity.y = float(imu_pkt[13]) * deg2rad
	msg.angular_velocity.z = float(imu_pkt[15]) * deg2rad
	msg.angular_velocity_covariance[0] = 0.0001
        msg.angular_velocity_covariance[4] = 0.0001
        msg.angular_velocity_covariance[8] = 0.0001
        
	msg.linear_acceleration.x = 0.0
	msg.linear_acceleration.y = 0.0
	msg.linear_acceleration.z = 9.8

	pub.publish(msg)
        sleep(sleep_duration)
        idx += 1
        
if __name__ == '__main__':
	try:
            talker()
  	except rospy.ROSInterruptException:
            pass
