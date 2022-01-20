#!/usr/bin/env python
# From: https://www.cse.sc.edu/~jokane/agitr/
# Translation to Python by Patricia Shaw
from opencv_apps.msg import MomentArrayStamped
import random
import rospy
import time
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import LaserScan
from tak17_ros360.srv import Shape, ShapeResponse

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


#!/usr/bin/env python
import tf2_ros
import rospy
import actionlib
from control_msgs.msg import (GripperCommandAction,
                              GripperCommandGoal)


#example based on cv_bridge tutorials
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped

flag = 0
flaginfo = ""
x = 0
y = 0
numberOfObjThrown = 0
objectDetected = False
TrashDetected = False
ReachedObject = False
ReachedTrash = False
depth = 0
area = 0
w=640
h=480
f = 554.254691191187
pose = [0.0,0,0]
bridge = CvBridge()
br = tf2_ros.TransformBroadcaster()

# callback method triggered when action is complete
def goalDone_cb(state, done):
	rospy.loginfo('goal done')

# callback method triggered when goal becomes active
def active_cb():
	rospy.loginfo('goal active')

# callback method triggered by feedback on action progress from action
def feedback_cb(fb):
	# feedback message contains the third set of variables defined in action message
	rospy.loginfo('goal progress...') #read feedback message

head_joint_names = ["head_tilt_joint", "head_pan_joint"]
head_joint_positions = [ 0.0,0.0]

def RedcallBackReceived(msg):
	global x
	global y
	global TrashDetected
	global area

	if len(msg.moments) > 0:		
		x = msg.moments[0].center.x
		y = msg.moments[0].center.y
		area = msg.moments[0].area
		rospy.loginfo("x, y and area are %.2f %.2f %2f", x, y, area)
		TrashDetected = True
	else:
		TrashDetected = False 
		

def BluecallBackReceived(msg):
	global x
	global y
	global objectDetected
	global area

	if len(msg.moments) > 0:		
		x = msg.moments[0].center.x
		y = msg.moments[0].center.y
		area = msg.moments[0].area
		rospy.loginfo("x, y and area are %.2f %.2f %2f", x, y, area)
		objectDetected = True
	else: 
		objectDetected = False



def cb_depthImage(image):
	global bridge, x, y, br
	# image msg obtained from callback message for topic
	global depth
	xCoor = int(x)
	yCoor = int(y)
	try:
		cv_image = bridge.imgmsg_to_cv2(image, "32FC1")
		# where x,y is the centre point from the published moment
		depth = cv_image[yCoor][xCoor]
		rospy.loginfo('Depth of point is %s m',depth)
		# For testing/verification:
		cv2.circle(cv_image, (xCoor, yCoor), 10, 0) # draw circle radius 10 at x,y
		cv2.imshow("Image window", cv_image) # display the image
		cv2.waitKey(3)
		if not math.isnan(depth):
			t = TransformStamped()
			t.header.stamp = rospy.Time.now()
			t.header.frame_id = "head_camera_depth_frame"
			t.child_frame_id = "target_object"
			t.transform.translation.x = depth
			t.transform.translation.y = -((xCoor-w/2)/f)*depth
			t.transform.translation.z = -((yCoor-h/2)/f)*depth
			q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0) # no rotation
			t.transform.rotation.x = q[0]
			t.transform.rotation.y = q[1]
			t.transform.rotation.z = q[2]
			t.transform.rotation.w = q[3]
			br.sendTransform(t)
	except CvBridgeError as e:
		print(e)

def pubvel():
	global x
	global y
	global flag	
	global flaginfo
	global objectDetected
	global TrashDetected
	global area
	global ReachedObject
	global ReachedTrash
	global pose
	global depth
	global numberOfObjThrown
	grabbedObject = False
	translationX = 0.0
	translationY = 0.0
	translationZ = 0.0
	rotationX = 0.0
	rotationY = 0.0
	rotationZ = 0.0
	rotationW = 0.0
	gripper_closed = 0.00
	gripper_open = 0.05
	pose[0] = 0.0
	
	# Initialise the ROS system and become a node
	rospy.init_node('publish_velocity')
	
	# Create a publisher object
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

	# Create a subscriber objecto
	rospy.Subscriber('/contour_moments_red/moments', MomentArrayStamped, RedcallBackReceived)
	rospy.Subscriber('/contour_moments_blue/moments', MomentArrayStamped, BluecallBackReceived)
	rospy.Subscriber('/head_camera/depth_registered/image_raw', Image, cb_depthImage)
	 # Create move group interface for a fetch robot
	move_group = MoveGroupInterface("arm_with_torso", "base_link")
	tfBuffer = tf2_ros.Buffer()
	iistener = tf2_ros.TransformListener(tfBuffer)
	head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	head_client.wait_for_server()
	rospy.loginfo("Waiting for gripper_controller...")
	gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
	gripper_client.wait_for_server()
	rospy.loginfo("...connected.")
	# A callback function. Executed each time a new pose message arrives
	trajectory = JointTrajectory()
	trajectory.joint_names = head_joint_names
	trajectory.points.append(JointTrajectoryPoint())
	trajectory.points[0].positions = pose 
	trajectory.points[0].velocities = [0.0] * len(pose)
	trajectory.points[0].accelerations = [0.0] * len(pose)
	trajectory.points[0].time_from_start = rospy.Duration(5.0)

	head_goal = FollowJointTrajectoryGoal()
	head_goal.trajectory = trajectory
	head_goal.goal_time_tolerance = rospy.Duration(5.0)
	head_client.send_goal(head_goal) #optioanly add callbacks, note fixed order
	#goalDone_cb, active_cb, feedback_cb) # 

	head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
   
	# Loop at 2Hz until the node is shutdown
	rate = rospy.Rate(10)
	startTime = rospy.get_time()
	while not rospy.is_shutdown():
		# Create and fill in the message.  The other four
		# fields,A which are ignored by turtlesim, default to 0.
		msg = Twist()
		#setting up the fine state machine
		if(numberOfObjThrown == 3): #if number of objects thrown equals 3 then task finished
			flag = 12
		elif(objectDetected == False): # if no object found keep turning
			flag = 0
		elif (x<300 and not ReachedObject): # if object found at the left of the image turn the robot to its right 
			flag = 1
		elif (x>360 and not ReachedObject): # if object found at the right of the image turn the robot to its left
			flag = 2
		elif (area>5 and area < 650):  #if depth greater than 0.6 still robot far from the target object
			flag = 3
		elif (objectDetected and y > 260): #checks if robot has got close enough to target and tilts the robot's head
				flag = 4
		elif ((depth<1)and(objectDetected==True) and not grabbedObject):#check if object detected after tilt0
				flag = 5
		elif(grabbedObject): #if grabbed object set the flag for robot
			flag = 6
		elif (x<300 and not ReachedTrash): # if trash found at the left of the image turn the robot to its right 
			flag = 7
		elif (x>360 and not ReachedTrash): # if object found at the right of the image turn the robot to its left
			flag = 8
		elif (TrashDetected):  #move towards the target until it disappers
			flag = 0
		elif (TrashDetected and y > 360):#reached close enough to the trash set the flag to tilt it's head
				flag = 10
		elif(ReachedTrash):#set the flag to throw the trash
			flag = "11" 
		#code to implement the finite state machine
		if(flag == 12): #end criteria met end the progrma exiting the loop
			flaginfor("End condition met for the robot, no further tasks for the robot")
			break
		elif (flag == 0): # if no object found keep turning
			flaginfo = "looking for objects"
			msg.angular.z = 0.75
		elif (flag == 1): # if object found at the left of the image turn the robot to its right 			
			flaginfo = "turning the robot slighlty to its right to center the object in its image"
			msg.angular.z = 0.5
		elif (flag == 2): # if object found at the right of the image turn the robot to its left	
			flaginfo = "turning the robot slighlty to its left to center the object in its image"
			msg.angular.z = -0.5 
		elif (flag == 3):  #move towards the target until it disappers		
			flaginfo = "move towards target"
			msg.linear.x = 0.5
		elif (flag == 4):
				flaginfo = "tilting it s head to view the target object"
				ReachedObject = True
				#msg.linear.x = -0.2
				pose[0] += 0.5
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

				rospy.loginfo("Setting tilt positions %s", pose)
				head_client.send_goal(head_goal) #optioanly add callbacks, note fixed order
				#goalDone_cb, active_cb, feedback_cb) # 
				head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
				try:
						trans = tfBuffer.lookup_transform('base_link', 'target_object', rospy.Time())
				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
						rate.sleep()
						continue
				translationX = trans.transform.translation.x
				translationY = trans.transform.translation.y
				translationZ = trans.transform.translation.z
				rotationX = trans.transform.rotation.x
				rotationY = trans.transform.rotation.y
				rotationZ = trans.transform.rotation.z
				rotationW = trans.transform.rotation.w
		elif (flag == 5):#thiw is elir statement is to grab the object
				flaginfo = "trying to pick the  target object"
				msg.linear.x = 0.0
				msg.angular.z = 0.0
				rospy.loginfo("Object Area after tilt =%d", area)
				reachPose = Pose(Point(translationX+0.1, translationY+0.1, translationZ+0.3),
                          Quaternion(rotationX, rotationY,rotationZ, rotationW))
				gripper_pose_stamped = PoseStamped()
				gripper_pose_stamped.header.frame_id = 'base_link'
				gripper_pose_stamped.header.stamp = rospy.Time.now()
				# Set the message pose
				gripper_pose_stamped.pose = reachPose
				gripper_frame = 'wrist_roll_link'
				# Move gripper frame to the pose specified
				move_group.moveToPose(gripper_pose_stamped, gripper_frame)
				result = move_group.get_move_action().get_result()
				gripper_goal = GripperCommandGoal()
				gripper_goal.command.max_effort = 10.0
				gripper_goal.command.position = gripper_open

				rospy.loginfo("Setting positions open...")
				gripper_client.send_goal(gripper_goal)
				gripper_client.wait_for_result(rospy.Duration(5.0))
				
				time.sleep(1.0)
				
				gripper_goal = GripperCommandGoal()
				gripper_goal.command.max_effort = 10.0
				gripper_goal.command.position = gripper_closed

				rospy.loginfo("Setting positions closed...")
				gripper_client.send_goal(gripper_goal)
				gripper_client.wait_for_result(rospy.Duration(5.0))

				grabedObject = True 
				if result:
					# Checking the MoveItErrorCode
					if result.error_code.val == MoveItErrorCodes.SUCCESS:
						rospy.loginfo("Hello there!")
						grabbedObject = True 
					else:
						# If you get to this point please search for:
						# moveit_msgs/MoveItErrorCodes.msg
						rospy.logerr("Arm goal in state: %s",
						move_group.get_move_action().get_state())
						grabbedObject = True 
				else:
					rospy.logerr("MoveIt! failure no result returned.")
					grabbedObject = True 
		elif(flag == 6):
			flaginfo = "looking for the trash"
			msg.angular.z = 0.75 #keep looking for the trash
		elif(flag == 7):
			flaginfo = "turning the robot slighlty to its right to center the trash in its image"
			msg.angular.z = 0.5 #center the robot's image
		elif(flag == 8):
			flaginfo = "turning the robot slighlty to its left to center the trash in its image"
			msg.angular.z = -0.5 #center the robot's image
		elif(flag == 9):
			flaginfo = "moving towards the trash"
			msg.linear.x = 0.2 #move towards the trash
		elif(flag == 10):
				flaginfo = "tilting its head to look at the bin"
				ReachedTrash = True
				pose[0] += 0.2
				tajectory = JointTrajectory()
				trajectory.joint_names = head_joint_names
				trajectory.points.append(JointTrajectoryPoint())
				trajectory.points[0].positions = pose 
				trajectory.points[0].velocities = [0.0] * len(pose)
				trajectory.points[0].accelerations = [0.0] * len(pose)
				trajectory.points[0].time_from_start = rospy.Duration(5.0)
				head_goal = FollowJointTrajectoryGoal()
				head_goal.trajectory = trajectory
				head_goal.goal_time_tolerance = rospy.Duration(0.0)

				rospy.loginfo("Setting tilt positions %s", pose)
				flag = "tilting its head to view the  target object"	
				head_client.send_goal(head_goal) #optioanly add callbacks, note fixed order
				#goalDone_cb, active_cb, feedback_cb) # 
				head_client.wait_for_result(rospy.Duration(6.0))  # specify timeout on waiting
		elif(flag == 11):
			#code to open up the gripper and throw the trash in the bin
				flaginfo = "Throwing trash"
				msg.linear.x = 0.0
				msg.angular.z = 0.0
				rospy.loginfo("Object Area after tilt =%d", area)
				try:
						trans = tfBuffer.lookup_transform('base_link', 'target_object', rospy.Time())
				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
						rate.sleep()
						continue
				trans.transform.translation.x
				trans.transform.translation.y
				trans.transform.translation.z
				trans.transform.rotation.x
				trans.transform.rotation.y
				trans.transform.rotation.z
				reachPose = Pose(Point(trans.transform.translation.x+0.2, trans.transform.translation.y+0.2, trans.transform.translation.z+0.3),
                          Quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))
				gripper_pose_stamped = PoseStamped()
				gripper_pose_stamped.header.frame_id = 'base_link'
				gripper_pose_stamped.header.stamp = rospy.Time.now()
				# Set the message pose
				gripper_pose_stamped.pose = reachPose
				gripper_frame = 'wrist_roll_link'
				# Move gripper frame to the pose specified
				move_group.moveToPose(gripper_pose_stamped, gripper_frame)
				result = move_group.get_move_action().get_result()
				gripper_goal = GripperCommandGoal()
				gripper_goal.command.max_effort = 10.0
				gripper_goal.command.position = gripper_open

				rospy.loginfo("Setting positions open...")
				gripper_client.send_goal(gripper_goal)
				gripper_client.wait_for_result(rospy.Duration(5.0))
				
				objectDetected = False
				time.sleep(1.0)
				
				if result:
					# Checking the MoveItErrorCode
					if result.error_code.val == MoveItErrorCodes.SUCCESS:
						rospy.loginfo("Hello there!")
					else:
						# If you get to this point please search for:
						# moveit_msgs/MoveItErrorCodes.msg
						rospy.logerr("Arm goal in state: %s",
									 move_group.get_move_action().get_state())
				else:
					rospy.logerr("MoveIt! failure no result returned.")
		else:
			rospy.logerr("something went wrong nothing detected for flag")#need to check what's causing the issue here.
		# Publish the message
		pub.publish(msg)
		
		# Send a message to rosout with the details
		rospy.loginfo("target object flag details = %s", flaginfo)
		
		# Wait until it's time for another iteration
		rate.sleep()


if __name__ == '__main__':
	try:
		pubvel()
	except rospy.ROSInterruptException:
		pass
