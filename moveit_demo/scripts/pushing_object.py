#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, SetModelState, SetModelStateRequest

class MoveRobot:

    def __init__(self):
        ## Initialize moveit_commander and rospy.
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_robot', anonymous=True)
        
        ## Instantiate a MoveGroupCommander object.
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.ball_position = geometry_msgs.msg.Pose().position

    def stop(self):
    
        ## When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()

    def goReady(self):
        self.group.set_named_target('ready')
        self.group.go()

    def get_model_state(self):

        ## Get the position of the ball
        get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        object_coordinates = GetModelStateRequest()
        object_coordinates.model_name = 'frc2016_ball'
        result = get_model_srv(object_coordinates)
        self.ball_position = result.pose.position
        print(self.ball_position)
    
    def set_model_state(self):

        ## Get the position of the ball
        set_model_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = SetModelStateRequest()
        model_state.model_state.model_name = 'frc2016_ball'
        model_state.model_state.pose.position.x = 0.7416
        model_state.model_state.pose.position.y = -0.070
        model_state.model_state.pose.position.z = 0.127
        model_state.model_state.pose.orientation.x = 0
        model_state.model_state.pose.orientation.y = 0
        model_state.model_state.pose.orientation.z = 0
        model_state.model_state.pose.orientation.w = 1

        result = set_model_srv(model_state)

    def push_ball(self):

        pose_target = geometry_msgs.msg.Pose()
        # pose_target.orientation.x = 0.924
        # pose_target.orientation.y = -0.382
        # pose_target.orientation.z = -0.001
        # pose_target.orientation.w = 0.0004
        pose_target.orientation = self.group.get_current_pose().pose.orientation
        pose_target.position.x = self.ball_position.x - 0.32
        pose_target.position.y = self.ball_position.y
        pose_target.position.z = self.ball_position.z + 0.1 # Move the end effector slightly above the ball to avoid collision
        self.group.set_pose_target(pose_target)

        ## Plan the trajectory to the goal
        plan = self.group.plan()

        ## Execute the plan
        self.group.go(wait=True)

        ## Get the current pose and modify it to push the ball
        current_pose = self.group.get_current_pose().pose
        current_pose.position.x += 0.3

        ## Set the new pose target
        self.group.set_pose_target(current_pose)

        ## Plan and execute the new trajectory
        plan = self.group.plan()
        self.group.go(wait=True)

    def main_loop(self):

        # 机械臂复位
        self.goReady()

        # 球复位
        self.set_model_state()

        # 获取物体的位置
        self.get_model_state()

        # 推球
        self.push_ball()

        # 机械臂复位
        self.goReady()

        # 球复位
        self.set_model_state()

        # stop
        self.stop()

if __name__ == "__main__":
    try:
        move_robot = MoveRobot()
        while not rospy.is_shutdown():
            move_robot.main_loop()
        move_robot.stop()
    except rospy.ROSInterruptException:
        pass
