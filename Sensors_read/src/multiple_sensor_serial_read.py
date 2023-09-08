#! /usr/bin/python3
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import serial
import time
import numpy as np
import pandas as pd

if __name__ == '__main__':
	ser = serial.Serial()
	ser.port = '/dev/ttyACM0'
	ser.baudrate = 38400
	ser.timeout = 1
	ser.open()
	print ("Port Open")
	bytesize = serial.EIGHTBITS
	parity = serial.PARITY_NONE
	stopbits = serial.STOPBITS_ONE
	pub = rospy.Publisher('chatter', Float64MultiArray, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	my_msg = Float64MultiArray()
	my_msg.layout.data_offset =  0 # no padding
	in_bytes = 0
	fl_start = 0
	fl_succ = 0
	sensor_data = []
	fl_sensor_data = []
	log_sensor_data = []
	log_sensor_data_time = []
	tmp = [0,0]
	sensor_data_both_bytes = []
	serial_bytes = []
	cnt_succ = 0
	cnt_trials = 0
	rate = rospy.Rate(10)
	print("Before While")
	while not rospy.is_shutdown():
		try:
			data = ser.read()
			dec_data = int.from_bytes(data, "little")
			if data == b'\x05' and fl_start == 0:
				fl_start = 1
				fl_succ = 0
				serial_bytes.append(data)
			elif data == b'\xfa':
				serial_bytes.append(data)
				fl_start = 0
				if in_bytes == 32:
					for i in range(16):
						tmp[0] = sensor_data[2*i].to_bytes(1, byteorder='big')
						tmp[1] = sensor_data[2*i + 1].to_bytes(1, byteorder='big')
						sensor_data_both_bytes.append(tmp[0] + tmp[1])
						sensor_data_both_bytes[i] = int.from_bytes(sensor_data_both_bytes[i], "little")
						fl_sensor_data.append(float(sensor_data_both_bytes[i]))
					log_sensor_data.append(fl_sensor_data)
					tmp_list = fl_sensor_data[:]
					tmp_list.append(rospy.get_time())
					log_sensor_data_time.append(tmp_list)					
					fl_succ = 1
				sensor_data = []
				sensor_data_both_bytes = []
				fl_sensor_data = []
				serial_bytes = []
				tmp_dec_data = []
				in_bytes = 0
			else:
				if fl_start == 1:
					in_bytes += 1
					sensor_data.append(dec_data)
					serial_bytes.append(data)

			if fl_succ == 1:
				my_msg.data = log_sensor_data[cnt_succ]
				rospy.loginfo(my_msg.data)
				pub.publish(my_msg)
				cnt_succ += 1

			cnt_trials += 1
			rate.sleep()

		except rospy.ROSInterruptException:
			ser.close()
			print('PROGRAM ENDED with ROS Exception')
			arr = np.asarray(log_sensor_data_time)
			df = pd.DataFrame(arr, columns = ["C%d" % (i) for i in range(17)])
			df.to_csv('/home/diego/catkin_ws/src/serial_py_icra2023/src/sensor_readings.csv')
			print('DATA SAVED')
			print('PROGRAM ENDED (FINALLY)')
			exit()

		except:
			ser.close()
			print('PROGRAM ENDED with random exception')
			arr = np.asarray(log_sensor_data_time)
			df = pd.DataFrame(arr, columns = ["C%d" % (i) for i in range(17)])
			df.to_csv('/home/diego/catkin_ws/src/serial_py_icra2023/src/sensor_readings.csv')
			print('DATA SAVED')
			print('PROGRAM ENDED')
			exit()

arr = np.asarray(log_sensor_data_time)
df = pd.DataFrame(arr, columns = ["C%d" % (i) for i in range(17)])
df.to_csv('/home/diego/catkin_ws/src/serial_py_icra2023/src/sensor_readings.csv') # Replace with your file location
print('DATA SAVED')
print('PROGRAM ENDED')
exit()
