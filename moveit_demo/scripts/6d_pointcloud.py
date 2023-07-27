#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from pcl import PointCloud, IterativeClosestPoint


#
# 6D pose estimation of an object
# Load the 3D model of the object
#
model_cloud = load_model()

def callback(data):
    # Convert the ROS point cloud message to a PCL point cloud
    observed_cloud = ros_to_pcl(data)

    # Create the ICP object
    icp = observed_cloud.make_IterativeClosestPoint()
    icp.setMaximumIterations(100)
    icp.setTransformationEpsilon(1e-8)

    # Perform the alignment
    converged, transform = icp.icp(observed_cloud, model_cloud)

    # If converged, print out the transformation (6D pose)
    if converged:
        rospy.loginfo("ICP converged.")
        rospy.loginfo("The obtained transformation is:")
        rospy.loginfo(transform)
    else:
        rospy.loginfo("ICP did not converge.")

def listener():
    rospy.init_node('pointcloud_listener', anonymous=True)
    rospy.Subscriber("/r200/camera/depth_registered/points", PointCloud2, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
