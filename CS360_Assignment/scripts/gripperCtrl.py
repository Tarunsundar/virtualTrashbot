#!/usr/bin/env python

import time

import rospy
import actionlib
from control_msgs.msg import (GripperCommandAction,
                              GripperCommandGoal)

gripper_closed = 0.00
gripper_open = 0.05


if __name__ == "__main__":
	rospy.init_node("gripper_control")

	rospy.loginfo("Waiting for gripper_controller...")
	gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
	gripper_client.wait_for_server()
	rospy.loginfo("...connected.")

	gripper_goal = GripperCommandGoal()
	gripper_goal.command.max_effort = 10.0
	gripper_goal.command.position = gripper_closed

	rospy.loginfo("Setting positions closed...")
	gripper_client.send_goal(gripper_goal)
	gripper_client.wait_for_result(rospy.Duration(5.0))

	time.sleep(1.0)

	gripper_goal = GripperCommandGoal()
	gripper_goal.command.max_effort = 10.0
	gripper_goal.command.position = gripper_open

	rospy.loginfo("Setting positions open...")
	gripper_client.send_goal(gripper_goal)
	gripper_client.wait_for_result(rospy.Duration(5.0))

	rospy.loginfo("...done")