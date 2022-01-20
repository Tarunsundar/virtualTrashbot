if __name__ == "__main__":
	rospy.init_node("head_control")

	rospy.loginfo("Waiting for head_controller...")
	
	rospy.loginfo("...connected.")


	for pose in head_joint_positions:

		if rospy.is_shutdown():
			break

		trajectory = JointTrajectory()
		trajectory.joint_names = head_joint_names
		trajectory.points.append(JointTrajectoryPoint())
		trajectory.points[0].positions = pose
		trajectory.points[0].velocities = [0.0] * len(pose)
		trajectory.points[0].accelerations = [0.0] * len(pose)
		trajectory.points[0].time_from_start = rospy.Duration(5.0)

		head_goal = FollowJointTrajectoryGoal()
		head_goal.trajectory = trajectory
		head_goal.goal_time_tolerance = rospy.Duration(0.0)

		rospy.loginfo("Setting positions...")
		head_client.send_goal(head_goal) #optioanly add callbacks, note fixed order
			#goalDone_cb, active_cb, feedback_cb) # 

		head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
		rospy.loginfo("...done")
