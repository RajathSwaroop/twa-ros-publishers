# twa-ros-publishers

Ros publishers for odom and imu with input coming from csv file.

IMU csv file format:
secs, 1614198459, nsecs, 72067299,roll, -0.0025771211115708788, pitch, -0.0043321937380371555, yaw, -0.005364942610031285, ang_vel_x, 0.0, ang_vel_y, 0.0, ang_vel_z, 0.0

Odom csv file format:
secs, 1614198459, nsecs, 723574000, x, -0.00034412386594340205, y, 0.0, q.x, 0.0, q.y, 0.0, q.z, -1.380097813807879e-08, q.w, 1.0                                                                           

To run the publishers:
-> Git clone into catkin_ws/src
-> catkin build twa_publishers
-> source ./devel/setup.bash
-> change path to csv file in imu_publisher and odom_publisher.py
-> rosrun twa_publishers imu_publisher.py
-> rosrun twa_publishers odom_publisher.py
-> data should be available on /odom and /imu
