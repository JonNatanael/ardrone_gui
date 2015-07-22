#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone_gui')
import rospy
from geometry_msgs.msg import Twist  

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

# define the default mapping between joystick buttons and their corresponding actions
ButtonEmergency = 0
ButtonLand      = 1
ButtonTakeoff   = 2
ButtonFlatTrim  = 3

# define the default mapping between joystick axes and their corresponding directions
AxisRoll        = 0
AxisPitch       = 1
AxisYaw         = 3
AxisZ           = 4

# define the default scaling to apply to the axis inputs. useful where an axis is inverted
ScaleRoll       = 1.0
ScalePitch      = 1.0
ScaleYaw        = 1.0
ScaleZ          = 1.0

# handles the reception of joystick packets
def ReceiveJoystickMessage(data):
	if data.buttons[ButtonEmergency]==1:
		rospy.loginfo("Emergency Button Pressed")
		controller.SendEmergency()
	elif data.buttons[ButtonLand]==1:
		rospy.loginfo("Land Button Pressed")
		controller.SendLand()
	elif data.buttons[ButtonTakeoff]==1:
		rospy.loginfo("Takeoff Button Pressed")
		controller.SendTakeoff()
	elif data.buttons[ButtonFlatTrim]==1:
		rospy.loginfo("Flat Trim Button Pressed")
		controller.SendFlatTrim()
	else:
		#print data.axes
		if data.axes[AxisRoll] != 0:
			if data.axes[AxisRoll]>0:
				rospy.loginfo("Rolling left")
			else:
				rospy.loginfo("Rolling right")
		elif data.axes[AxisPitch] != 0:
			if data.axes[AxisPitch]>0:
				rospy.loginfo("Moving forward")
			else:
				rospy.loginfo("Moving backward")
		elif data.axes[AxisYaw] != 0:
			if data.axes[AxisYaw]>0:
				rospy.loginfo("Turning left")
			else:
				rospy.loginfo("Turning right")	
		elif data.axes[AxisZ] != 0:
			if data.axes[AxisZ]>0:
				rospy.loginfo("Moving up")
			else:
				rospy.loginfo("Moving down")

		controller.SetCommand(data.axes[AxisRoll]/ScaleRoll,data.axes[AxisPitch]/ScalePitch,data.axes[AxisYaw]/ScaleYaw,data.axes[AxisZ]/ScaleZ)


# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('test')

	pubCommand = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
	rospy.sleep(1)

	c = Twist()
	c.angular.z = 5



	print "Turning left"
	pubCommand.publish(c)
	#rospy.sleep(1)
	#pubCommand.publish(c)

	
	#rospy.spin()