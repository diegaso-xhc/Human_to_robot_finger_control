#!/usr/bin/env python3

# User sample code that calibrates the sensor via a ROS message, recovers the polar coordinates sensor values and display them

import rospy
import std_msgs.msg
import serial, sys
from sensor_pkg.msg import *
import time

# Callback function definition
def callback(sensor_data):
    # Initialize a list of lone_sensor messages to store the data that are read
    sensor_values = [lone_sensor() for i in range(sensor_data.length)]
    for i in range(sensor_data.length):
        if sensor_data.data[i].is_present == False:             # Check if the sensor is present
            sensor_values[i].id = None                          # if not : set id to None, sensor will be displayed as "Sensor None"
        else:
            sensor_values[i] = sensor_data.data[i]              # If sensor is present, then copy the informations in the lone_sensor message
        # Then print each sensor with its polar coordinates
        print("\t Sensor {} \n".format(sensor_values[i].id))
        print("mN_Value \t \t rad_Pitch \t \t \t rad_Yaw \n")
        print("{} \t {} \t {} \n".format(sensor_values[i].abs, sensor_values[i].pitch, sensor_values[i].yaw))

# Initialize ros node
rospy.init_node('user_listener_2', anonymous = True)

# Initialize a publisher to user_command to be able to send command to the sensors
pub = rospy.Publisher('user_command', user_command, queue_size = 10)

# Defining a user_command message to fill it with the needed Command
command = user_command()

# Setting the 'calibrate' field to True
command.calibrate = True

# Let 1s to ROS for the Node's initialization before publishing
time.sleep(1)

# Publish the command to the user_command topic.
pub.publish(command)

# Subscribe to the AllSensors Topic, in which informations read about sensors are published
rospy.Subscriber("AllSensors", AllSensors, callback)

# Keeps the node listening until you stop the script manually
rospy.spin()
