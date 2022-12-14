#!/usr/bin/env python

import cv2
#import pcl
import pypcd
from pypcd import pcl
#import pcl_helper
import rospy
import rosbag
import roslib
import sys, getopt
import os
import numpy as np
import operator
import threading
import sensor_msgs
import sensor_msgs.point_cloud2 as pc2
import ros_numpy


from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

import coord_conv

check_firstIMU = True
euler_list_prev = (0.0, 0.0, 0.0)
euler_list = (0.0, 0.0, 0.0)
diff_att = (0.0, 0.0, 0.0)
diff_att_list = [0.0, 0.0, 0.0]
imu_time = 0
line_num = 17 # Lidar line No.

class rotations:
    def quaternion_rotation_matrix(Q):
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrixc
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix

class mapping:

    def lidar_callback(lidar_data):

        global euler_list
        global euler_list_prev
        global diff_att
        global diff_att_list

        lidar_timestamp = lidar_data.header.stamp
    #    print("lidar time: ", lidar_timestamp)

        #pub_lidar = rospy.Publisher('/ifws_robot/points', PointCloud2, queue_size=1000)
         
        # Update imu data
        rospy.Subscriber('/imu/data', Imu, mapping.imu_callback)
        
        

        # Process & Diaplay
        mapping.processing(lidar_data)
        euler_list_prev = euler_list
        return

    
    def imu_callback(imu_data):

        global euler_list
        global euler_list_prev
        global check_firstIMU
        

        euler_list, rot_matrix= mapping.imu_to_rotation(imu_data)
        
        if check_firstIMU :
            euler_list_prev = euler_list
            check_firstIMU = False

        
        
        
        
        #pub_imu = rospy.Publisher('/ifws_robot/cmd_vel', Twist, queue_size=1000)

        #vel = Twist()
        #vel.linear.x  = euler_list[1] # Pitch
        #vel.angular.z = euler_list[0] # Roll

        #pub_imu.publish(vel)
        
        return 


    def imu_to_rotation(imu_data):
        imu_timestamp = imu_data.header.stamp
        #print("imu time: ", imu_timestamp)
        global imu_time
        imu_time = imu_timestamp

        quat_x = imu_data.orientation.x
        quat_y = imu_data.orientation.y
        quat_z = imu_data.orientation.z
        quat_w = imu_data.orientation.w

        quaternion_list= [quat_x, quat_y, quat_z, quat_w]

        # Quarternion to Euler
        euler_list = euler_from_quaternion(quaternion_list)
        # Quarternion to Rotation matrix
        rot_matrix = rotations.quaternion_rotation_matrix(quaternion_list)
        
        return euler_list, rot_matrix


    def msgToPCL(lidar_data):

        lidar_data.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
        offset_sorted = {f.offset: f for f in lidar_data.fields}
        lidar_data.fields = [f for (_, f) in sorted(offset_sorted.items())]

        # Conversion from PointCloud2 msg to np array.
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(lidar_data, remove_nans=True)
        pc_pcl = pcl.PointCloud(np.array(pc_np, dtype=np.float32))
        return pc_np, pc_pcl  # point cloud in numpy and pcl format

    def processing(lidar_data):

        lock.acquire()

        diff_att = tuple(map(operator.sub, euler_list, euler_list_prev))
        diff_att_list = list(diff_att)

    #    print("imu time: ", imu_time)
    #    print("euler_list : ", euler_list)
    #    print("euler_list_prev : ", euler_list_prev)
    #    print("diff_att : ", diff_att_list)

        # -pi ~ pi jumping
        cnt = 0
        for i in diff_att_list:
            if i > np.pi:
                i = i - 2*np.pi
            elif i < np.pi:
                i = i + 2*np.pi

            diff_att_list[cnt] = i
            cnt = cnt + 1
    #    print("diff_att_test: ", diff_att_list)
        
        cnt = 0
        delta_att = [0.0, 0.0, 0.0]
        for j in diff_att_list:
            delta_att[cnt] = j / line_num
            cnt = cnt+1

    #    print("delta_att : ", delta_att)
            
        # Update lidar data
        pc_np, pc_pcl = mapping.msgToPCL(lidar_data)

        print(pc_np)
        lock.release()

        return


if __name__ == '__main__':
    print()
    print('infoworks test')
    print()

    lock = threading.Lock()

    # Read rostopic
    rospy.init_node("mapping")
    rospy.Subscriber('/ifwslidar_pcl', PointCloud2, mapping.lidar_callback)
   # rospy.Subscriber('/imu/data', Imu, mapping.imu_callback)


    frame_id_ = "world"
    child_frame_id_ = "odom"

    rospy.spin()

