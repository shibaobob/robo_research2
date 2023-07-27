#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

def model_states_callback(msg):
    # Replace 'object_name' with the name of your object in Gazebo
    # name: [ground_plane, coke_can, beer, cricket_ball, frc2016_ball, panda]
    object_name = 'frc2016_ball'
    
    if object_name in msg.name:
        index = msg.name.index(object_name)
        pose = msg.pose[index]
        rospy.loginfo('6D pose of {}:'.format(object_name))
        rospy.loginfo('Position: x={}, y={}, z={}'.format(pose.position.x, pose.position.y, pose.position.z))
        rospy.loginfo('Orientation: x={}, y={}, z={}, w={}'.format(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    else:
        rospy.logwarn('{} not found in Gazebo.'.format(object_name))

def main():
    rospy.init_node('object_pose_collector_node', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
