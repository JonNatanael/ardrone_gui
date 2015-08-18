#!/usr/bin/env python

# The Joystick Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Joystick Control"
# https://github.com/mikehamer/ardrone_tutorials

# This controller implements the base DroneVideoDisplay class, the DroneController class and subscribes to joystick messages

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_gui')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Import the joystick message
from sensor_msgs.msg import Joy

# Finally the GUI libraries
from PySide import QtCore, QtGui

# define the default mapping between joystick buttons and their corresponding actions
ButtonEmergency = 0
ButtonLand      = 1
ButtonTakeoff   = 2
ButtonFlatTrim  = 3
ButtonUtil = 4

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
	elif data.buttons[ButtonUtil]==1:
		rospy.loginfo("Util Button Pressed")
		controller.SendUtil()
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
	rospy.init_node('ardrone_joystick_controller')

	# Next load in the parameters from the launch-file
	ButtonEmergency = int (   rospy.get_param("~ButtonEmergency",ButtonEmergency) )
	ButtonLand      = int (   rospy.get_param("~ButtonLand",ButtonLand) )
	ButtonTakeoff   = int (   rospy.get_param("~ButtonTakeoff",ButtonTakeoff) )
	ButtonFlatTrim   = int (   rospy.get_param("~ButtonFlatTrim",ButtonFlatTrim) )
	ButtonUtil   = int (   rospy.get_param("~ButtonUtil",ButtonUtil) )

	AxisRoll        = int (   rospy.get_param("~AxisRoll",AxisRoll) )
	AxisPitch       = int (   rospy.get_param("~AxisPitch",AxisPitch) )
	AxisYaw         = int (   rospy.get_param("~AxisYaw",AxisYaw) )
	AxisZ           = int (   rospy.get_param("~AxisZ",AxisZ) )
	ScaleRoll       = float ( rospy.get_param("~ScaleRoll",ScaleRoll) )
	ScalePitch      = float ( rospy.get_param("~ScalePitch",ScalePitch) )
	ScaleYaw        = float ( rospy.get_param("~ScaleYaw",ScaleYaw) )
	ScaleZ          = float ( rospy.get_param("~ScaleZ",ScaleZ) )

	controller = BasicDroneController()

	subJoystick = rospy.Subscriber('/joy', Joy, ReceiveJoystickMessage)
	
	rospy.spin()