#!/usr/bin/env python3

# User sample code that sends a command to calibrate the sensor and sets its frequency to a new one.

import rospy
import std_msgs.msg
from sensor_pkg.msg import *
import time

DESIRED_FREQUENCY = 20          #20Hz, can be changed with values between 1 and 50

# Initialize ros node
rospy.init_node('user_talker', anonymous = True)
# Initialize a publisher to user_command to be able to send command to the sensors
pub = rospy.Publisher('user_command', user_command, queue_size = 10)
# Defining a user_command message to fill it with the needed Command
command = user_command()
# Setting the 'calibrate' field to True
command.calibrate = True
# Setting the 'set_frequency' field to True
command.set_frequency = True
# Setting the 'frequency' field to the desired frequency
command.frequency = DESIRED_FREQUENCY
# Let 1s to ROS for the Node's initialization before publishing
time.sleep(1)
# Publish the command to the user_command topic. It will calibrate the sensors and set the frequency to the desired one.
pub.publish(command)
