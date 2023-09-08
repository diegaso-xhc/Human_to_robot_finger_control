#!/usr/bin/env python3
import rospy
import std_msgs.msg
import serial, sys
from sensor_pkg.msg import *
import numpy as np
import time

# Getting the number of sensors through the ros parameters initilized in the launch file
try:
    SENSOR_NUMBER = rospy.get_param("/seed_fts3/sensor_number")
# If the ROS parameter doesn't exist, set it to 5 (default value)
except KeyError:
    print("Could not find the number of sensors, setting to default (5)")
    SENSOR_NUMBER = 5
    pass

# Constant the will be used later to check if sensor's data have the right length
SUPPOSED_ARRAY_LENGTH = SENSOR_NUMBER*3+4 # 3 Directions/sensor, 2 timestamps, starting carac, eol.

# Getting port via ROS parameters
try:
    port = rospy.get_param("/seed_fts3/port")
    print(port)
# If the ROS parameter doesn't exist, set the port to /dev/ttyUSB0 (default value)
except KeyError:
    print("Could not find port parameter, setting to default port (/dev/ttyUSB0)")
    port = '/dev/ttyUSB0'
    pass

# Declaring sensor baudrate
baudrate = 1000000

# Open port
try:
    sensor_read = serial.Serial(port, baudrate, timeout = 1.5, write_timeout = 1)
except serial.SerialException:
    rospy.logerr("Could not open serial port %s", port)
    quit()

# Initializing the Ros listener callback to read user_command and write back for the sensors
def user_command_callback(user_command):
                                                            # If user sets calibrate to True
    if user_command.calibrate == True:
        sensor_read.write('calibrate\r\n'.encode('ascii'))  # Then send the calibrate command to the sensor
        rospy.loginfo("Sensors calibrated")

    if user_command.setepoch == True:                       # If the user sets 'setepoch' field to true
    # Retrieving timestamp values from the command
        secs = user_command.epoch_sec
        msecs = user_command.epoch_msec
    # Concatenate seconds and milliseconds into a string
        timestamp = secs + ',' + msecs
        set_epoch_command = 'setepoch,'+ secs + ',' + msecs + '\r\n'
    # Send the command to the sensor via serial port
        sensor_read.write(set_epoch_command.encode('ascii'))
    # Log the info that the command has been sent
        rospy.loginfo("Command sent to sent epoch to %s" % timestamp)
        time.sleep(0.1)

    if user_command.diagnosis_request == True:              # If the user requests for a diagnosis
        # Send the dumpsensors command to get informations
        sensor_read.write('dumpsensors\r\n'.encode('ascii'))
        time.sleep(0.1)
        # Send the command to get the hardware version of the sensors
        sensor_read.write('getversion\r\n'.encode('ascii'))
        time.sleep(0.1)
        # Send the command to get the software version of the sensors
        sensor_read.write('softwareversion\r\n'.encode('ascii'))
        time.sleep(0.1)
        # Send the command to get info about the board
        sensor_read.write('dumponboard\r\n'.encode('ascii'))
        time.sleep(0.1)
        # Send the resume command to get the sensors go back to their usual job
        sensor_read.write('resume\r\n'.encode('ascii'))

    if user_command.set_frequency == True:
        period_ms = (1 / user_command.frequency) * 1000          # User sets frequency, converting to ms to send the message to the sensors
        print("requested period is :" + str(period_ms))
        set_freq_str = "setperiod," + str(period_ms)
        sensor_read.write(set_freq_str.encode('ascii'))
        # Check if the requested frequency can be handled by the sensors
        if period_ms >= 20:
            rospy.loginfo("Sensor frequency set to %s" % str(user_command.frequency))
        else:
            rospy.loginfo("Tried to change frequency above the limit. Try again with a value between 1 and 50")
        time.sleep(0.1)

    if user_command.raw_string == True:         # If the user wants to send a custom command
        raw = user_command.raw                  # Read the command
        sensor_read.write(raw.encode('ascii'))  # Send it to the sensors
        rospy.loginfo("Command %s sent" % user_command.raw)
        time.sleep(0.1)

# Initializing the Ros publisher Node for sensor data
pub = rospy.Publisher('AllSensors', AllSensors, queue_size = 10)    #Type of message to be sent : AllSensors
rospy.init_node('seed_fts3', anonymous=True)                        # Initialize a node named seed_fts3
rospy.Subscriber('user_command', user_command, user_command_callback)
r = rospy.Rate(50)                                                  # 50Hz



# Class sensor : Stores sensors attributes to be sent with ROS messages
class Sensor:

    def __init__(self, id=0, fx=0, fy=0, fz=0):
        self.id = id
        self.fx = fx
        self.fy = fy
        self.fz = fz

    abs = 0
    yaw = 0.0
    pitch = 0.0
    is_present = True
    is_3D = True

# Defining a function to convert Cartesian coordinates to spherical coordinates
def cart2sph(x, y, z):
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.arctan2(hxy, z)
    az = np.arctan2(y, x)
    return r, el, az

#Initializing a list of sensors
sensors = [Sensor() for i in range(SENSOR_NUMBER)]


# Calibrate sensors
time.sleep(0.1)
sensor_read.write('calibrate\r\n'.encode('ascii'))
time.sleep(0.1)
# Set the sensor's epoch
secs = rospy.get_rostime().secs
msecs = rospy.get_rostime().nsecs / 1000000
msecs = int(msecs)
set_epoch_command = 'setepoch,'+str(secs)+','+str(msecs)+'\r\n'
sensor_read.write(set_epoch_command.encode('ascii'))

# Initializing the messages to send
lone_sensor_msgs = [lone_sensor() for i in range(SENSOR_NUMBER)]    # Initialize an array of lone_sensor messages
final_msg = AllSensors()                                            # Initialize an AllSensors message to fill in later




#Parse read data into the Sensors list. Input : str list of the data read
def parse_data_into_obj(data):
    #Check if the current line is correct
    if data[0] == '@':
        #Defining timestamp with the 2 first data : time (s) and time (ms)
        timestamp = data[1] + "," + data[2]
        for i in range(SENSOR_NUMBER):
            if (data[5 + 3 * i] == ''):                                                         # If Fx or Fy field is empty on the data stream
                sensors[i].id = i                                                               # Then put all fields to 0
                sensors[i].fx = 0
                sensors[i].fy = 0
                sensors[i].fz = 0
                sensors[i].is_present = False                                                   # And warn that there is a sensor missing
            elif (data[5 + 3 * i] != '' and (data[3 + 3 * i] == '' or data[4 + 3 * i] == '')):  # If only the fz field is filled
                sensors[i].id = i
                sensors[i].fx = 0
                sensors[i].fy = 0
                sensors[i].fz = fz
                sensors[i].is_3D = False                                                        # Then it's not a 3D sensor but a 1D sensor
            else:                                                                               # Usual case : this is a 3D sensor
            # Fill all the attributes
                fx = int(data[3 + 3 * i])
                fy = int(data[4 + 3 * i])
                fz = int(data[5 + 3 * i])
                sensors[i].id = i
                sensors[i].fx = fx
                sensors[i].fy = fy
                sensors[i].fz = fz
                r, theta, phi = cart2sph(fx, fy, fz)    # Convert cartesian coordinates to polar coordinates
                sensors[i].abs = r                      # Fill in the polar coordinates attributes
                sensors[i].pitch = phi
                sensors[i].yaw = theta
        return timestamp
    elif data[0] == '#OK':          # If the line starts with a #OK, then it's a response to a command
        message = ""
        for elem in data:           # Concatenate the message into a string
            message += elem
        rospy.loginfo(message)      # Log it into the ROS logs
        return None
    else:                           # If the line doesn't start neither with a @ nor a #OK then it's an error
        # Display the read data
        try:
            print("Serial data is :", data)
        except:
            pass
        # Log a ROS waring
        rospy.logwarn("A sensor line could not be read : wrong start character" + str(data))
        return None


#Parse input data into a str list. Input : Serial with the correct port and baudrate
def parse_data(serial_sensor):
    #Read the first line

    line = serial_sensor.readline()
    if line:
        #Decode the first line from bytes to str
        line_str = line.decode("utf-8")
        #Split the line into a str list
        data = line_str.split(",")
        #Check if all the sensors are sending data
        if len(data) == SUPPOSED_ARRAY_LENGTH:
            #Call to the function that put the str list into Sensor objects attributes
            timestamp = parse_data_into_obj(data)
            #timestamp = parse_data_into_obj(fake_data)
        else:
            #Throw warning
            print("Weird length")
            rospy.logwarn("Weird length")
            #Filter the list : finding empty elements and removing them
            #new_list = list(filter(lambda x: (x != ""),data))
            #print(new_list)
            #Call to the function that put the str list into Sensor objects attributes
            timestamp = parse_data_into_obj(data)
    else:
        return None
    return timestamp

# Main loop
while not rospy.is_shutdown():
    timestamp = parse_data(sensor_read)     # Parses the serial data into the list of Sensor objects
    # Loop that goes through the list of Sensor objects
    # Then fills each messages of the lone_sensor_msgs list with the Sensor objects' attributes
    for i in range(SENSOR_NUMBER):
        lone_sensor_msgs[i].id = sensors[i].id
        lone_sensor_msgs[i].fx = sensors[i].fx
        lone_sensor_msgs[i].fy = sensors[i].fy
        lone_sensor_msgs[i].fz = sensors[i].fz
        lone_sensor_msgs[i].abs = sensors[i].abs
        lone_sensor_msgs[i].yaw = sensors[i].yaw
        lone_sensor_msgs[i].pitch = sensors[i].pitch
        lone_sensor_msgs[i].is_present = sensors[i].is_present
    # Fill in the AllSensors message
    final_msg.length = SENSOR_NUMBER            # Fill the length
    final_msg.header.stamp = rospy.Time.now()   # Get the timestamp
    final_msg.data = lone_sensor_msgs           # Fill the data list with the lone_sensor_msgs list
    # Publish the AllSensors message
    pub.publish(final_msg)
    try:
        r.sleep()       # Sleep for 20ms (highest frequency of the sensors)
    except rospy.ROSInterruptException:
        rospy.logwarn("Interrupt Exception on rate.sleep()")
        pass
