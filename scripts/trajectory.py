#!/usr/bin/env python2
#
# Code to send trajectory to the command node by sending setpoints at a certain rate.
# By running the main fonction (waithere), User can decide what to do depending on the drone state (armed or not, in air or not)

import rospy
import time
import math
import numpy as np
import tkinter
from tkinter import Tk
from tkFileDialog import askopenfilename #Used to retrieve a file name from an explorer
from tkinter import filedialog
from geometry_msgs.msg import Pose, Quaternion, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from tf.transformations import quaternion_from_euler

armed=False

def arm(armed):
	""" This function checks if the drone is already armed. If not, It asks user if he wants to arm it and does so input is the state of the drone"""
	if armed == True:
		return #Drone is already armed, nothing to do
	else:
		arm_command=raw_input('Do you want to arm the vehicle ? (y)') #Make sure the user wants to arm the drone
		try:
			arm_command = str(arm_command)
			pass
		except ValueError:
			print('Enter y or n please')
			return
		else:
			if arm_command =='y':
				arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool, 10)
				arming(True)
				pass
			else:
				return

def callback(state):
	""" This callback fonction returns the arming state of the drone """
	global armed
	armed = state.armed

def land(input_pub):
	"""This funtion is used to tell the drone to land. For this, the setpoint is pointed at altitude 0. """
	landing=raw_input('Do you want to land ? (y)') #Make sure the user wants to land
	try:
		landing=str(landing)
		pass
	except:
		print('Enter y or n please')
		return
	else:
		setpoint=Point(0,0,0) #Setpoint at 0 altitude
		yaw_deg=0
		yaw=math.radians(yaw_deg)
		angle=quaternion_from_euler(0,0,yaw)
		setorientation=Quaternion(*angle)
		input_pose=Pose(setpoint,setorientation)
		input_pub.publish(input_pose)
		return setpoint


def takeoff(input_pub):
	"""This funtion is used to tell the drone to take off. For this, the setpoint is pointed at an altitude different than 0 """
	if armed == False: #If drone is not armed, go to arm function
		arm()
	else:
		to=raw_input('Do you want to take off ? (y)')
		try:
			to=str(to)
			pass
		except ValueError:
			print('Please enter y or n')
		else:
			if to == 'y':
				setpoint=Point(0,0,1.30) #Last parameter is the altitude.
				yaw_deg=0
				yaw=math.radians(yaw_deg)
				angle=quaternion_from_euler(0,0,yaw)
				setorientation=Quaternion(*angle)
				input_pose=Pose(setpoint,setorientation)
				input_pub.publish(input_pose)
				return setpoint
			else:
				return

def trajectory(input_pub):
	"""This function withdraws a series of points to be reached in order to establish a trajectoryself.
	The points are taken from a csv file chose by the user. The points should be in the form :(x,y,z,yaw)
	The points are published at a 10Hz rate to match with the commande node."""
	Tk().withdraw()
	filename = askopenfilename(initialdir='~/Downloads')
	print('entering trajectory mode')
	points = np.genfromtxt(filename,delimiter = ',')
	print('trajectory loaded')


	for row in points:
		print('reading point')
		setpoint=Point(*row[:3])
		yaw_deg=row[3]
		yaw=math.radians(yaw_deg)
		angle=quaternion_from_euler(0,0,yaw)
		setorientation=Quaternion(*angle)
		input_pose=Pose(setpoint,setorientation)
		print('sending point')
		input_pub.publish(input_pose)
		print('point sent:',input_pose)
		time.sleep(0.05)

	return setpoint


def waithere():
	"""Main function.  It inits the node, the publisher for the points and the subscriber for the drone state
	It asks what the user wants to do based on the drone state"""


	print('Input node initiating')


	rospy.Subscriber('/mavros/state',State, callback) #subscribe to the state topic and call the callback function to get the arming state of the drone

	input_pub=rospy.Publisher('trajectory_sender',Pose, queue_size=10) #Publish the pose to the topic on which the command node will subscribe to send it to the drone
	rospy.init_node('trajectory_input',anonymous=False)

	rate = rospy.Rate(0.2)

	while not rospy.is_shutdown():


		if armed == True: #These conditions determine which function to run depdending on the user will and the drone state
			choice = raw_input('Do you want to send a trajectory (1), to land (2) or to takeoff (3) ? Enter 1, 2 or 3')
			try:
				choice=int(choice)
				pass
			except ValueError:
				print('Enter 1 to send a trajectory, 2 to land or 3 to takeoff')
				#continue
			else:
				if choice == 1:
					trajectory(input_pub)
				elif choice == 2:
					land(input_pub)
				elif choice == 3:
					takeoff(input_pub)
				else:
					print('Enter 1 to send a trajectory or 2 to land')
					#continue
		else:
			arm(armed)

		rate.sleep()

if __name__ == '__main__':
	try:
		waithere()
	except rospy.ROSInterruptException:
		pass
