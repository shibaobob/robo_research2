#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist, Quaternion
import math

from geometry_msgs.msg import Twist

class MoveGazeboModel:
    def __init__(self):
        rospy.init_node('move_gazebo_model', anonymous=True)
        self.client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.rate = rospy.Rate(100)

        self.position_x_pub = rospy.Publisher('position_x', Twist, queue_size=1)
        rospy.sleep(4)

        self.pointA = [1, -0.5, 0.15]
        self.pointB = [1, 0.5, 0.15]
        self.duration = 6500
        self.time_t = 0

    def run(self):
        des_model_state = ModelState()
        des_model_state.model_name = "frc2016_ball"   
        pose = Pose()
        twist = Twist()

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        while not rospy.is_shutdown():

            alpha = (math.cos(self.time_t * math.pi / self.duration) + 1) / 2

            pose.position.x = (1 - alpha) * self.pointA[0] + alpha * self.pointB[0]
            pose.position.y = (1 - alpha) * self.pointA[1] + alpha * self.pointB[1]
            pose.position.z = (1 - alpha) * self.pointA[2] + alpha * self.pointB[2]

            position_x_msg = Twist()
            position_x_msg.linear.x = pose.position.x
            position_x_msg.linear.y = pose.position.y
            position_x_msg.linear.z = pose.position.z
            self.position_x_pub.publish(position_x_msg)

            des_model_state.pose = pose
            des_model_state.twist = twist
            des_model_state.reference_frame = "world"

            try:
                self.client.call(des_model_state)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

            self.rate.sleep()
            self.time_t += 1/self.rate.sleep_dur.to_sec()

            if self.time_t >= self.duration:
                self.time_t -= self.duration

if __name__ == '__main__':
    try:
        mover = MoveGazeboModel()
        mover.run()
    except rospy.ROSInterruptException:
        pass
