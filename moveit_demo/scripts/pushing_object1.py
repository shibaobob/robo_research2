#!/usr/bin/python
# -*- coding: utf-8 -*-


import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

def move_robot():
    ## Initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot', anonymous=True)

    ## Instantiate a RobotCommander object.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.
    group = moveit_commander.MoveGroupCommander("panda_arm")

    ## Get the position of the ball
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    object_coordinates = GetModelStateRequest()
    object_coordinates.model_name = 'frc2016_ball'
    result = model_coordinates(object_coordinates)
    ball_position = result.pose.position

    ## Specify target pose for end effector link.
    ## This will need to be modified to move to the correct location to push the ball.
    ##########################################
    ## panda_arm initial pose:
    ## Position: x=0.554776239293, y=-1.37316782989e-05, z=0.623175519546
    ## Orientation: x=0.923955399622, y=-0.382498165105, z=-0.00116619201324, w=0.000461731637235
    ## 6D pose of frc2016_ball:
    ## Position: x=0.741593, y=-0.07074, z=0.126999995829
    ## Orientation: x=0.0, y=0.0, z=0.0, w=1.0
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = 0.924
    pose_target.orientation.y = -0.382
    pose_target.orientation.z = -0.001
    pose_target.orientation.w = 0.0004
    pose_target.position.x = ball_position.x - 0.32
    pose_target.position.y = ball_position.y
    pose_target.position.z = ball_position.z + 0.1 # Move the end effector slightly above the ball to avoid collision
    group.set_pose_target(pose_target)

    ## Plan the trajectory to the goal
    plan = group.plan()

    ## Execute the plan
    group.go(wait=True)

    ## Get the current pose and modify it to push the ball
    current_pose = group.get_current_pose().pose
    current_pose.position.x += 0.3

    ## Set the new pose target
    group.set_pose_target(current_pose)

    ## Plan and execute the new trajectory
    plan = group.plan()
    group.go(wait=True)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
