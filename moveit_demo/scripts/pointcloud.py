#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
import os


# def callback(data):
#     pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
#     points = list(pc)
#     rospy.loginfo("Received {0} points in the point cloud".format(points))

# def listener():
#     rospy.init_node('pointcloud_listener', anonymous=True)
#     rospy.Subscriber("/r200/camera/depth_registered/points", PointCloud2, callback)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()




# Counter for the saved files
file_counter = 0

def callback(data):
    global file_counter
    pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(pc))
    rospy.loginfo("Received {0} points in the point cloud".format(len(points)))

    # Save the points to a .npy file
    filename = 'point_cloud_{}.npy'.format(file_counter)
    np.save(filename, points)

    file_counter += 1

def listener():
    rospy.init_node('pointcloud_listener', anonymous=True)
    rospy.Subscriber("/r200/camera/depth_registered/points", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

    