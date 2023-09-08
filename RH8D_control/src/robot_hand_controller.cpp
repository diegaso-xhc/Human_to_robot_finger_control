#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>

using namespace std;
unsigned int microsecond = 1000000;

std_msgs::Float64 h_stretch; // global variable
std_msgs::Float64 h_contact; // global variable
std_msgs::Float64 r_stretch; // global variable
std_msgs::Float64 r_contact; // global variable

void chatter64Mult(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // This is a function that assigns 4 of the 16 channels read to sensor variables
  h_stretch.data = msg->data[14];
  h_contact.data = msg->data[15];
  r_stretch.data = msg->data[10];
  r_contact.data = msg->data[11];  
}

// The following lines help with saving a file with read data
std::ofstream myfile;
void record_data(float x[6], int n)
{
    myfile << std::setprecision(n) << x[0] << "," << x[1] << "," << x[2] << "," << x[3] << "," << x[4] << "," << x[5] << "\n";    
}

// Since for our application we used a sensor to measure the PIP angle of a finger, we use a calibratio curve.
float get_h_angle(float x)
{
    // This function calculares the angle of the PIP joint of the human finger
    float y;    
    x = (x - 64568)/64568; // Callibration on the 31.08.2022 (PIP)
    y = (-2.406e+11)*x*x*x + (-3.787e+07)*x*x + (-2.644e+04)*x + 8.177; // Callibration on the 31.08.2022 (PIP)

    if (y < 5)
        y = 5.0f;
    else if (y > 75)
        y = 75.0f;
    return y;
}

//
float get_target_sensor_value(float x)
{
    // This function maps the desired angle on the robot hand to a value which can be commanded to the hand.
    float y;
    y = (-3.555e-05)*x*x*x*x + 0.007962*x*x*x + (-0.5668)*x*x + 10.29*x + 6.472e+04 -80;
    return y;
}


int main(int argc, char **argv)
{
  // Setting all ROS publishers and rate
  ros::init(argc, argv, "robot_hand_controller");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatter64Mult);
  ros::Publisher W_r_controller = n.advertise<std_msgs::Float64>("/W_r_controller/command", 1000);
  ros::Publisher W_a_controller = n.advertise<std_msgs::Float64>("/W_a_controller/command", 1000);
  ros::Publisher W_f_controller = n.advertise<std_msgs::Float64>("/W_f_controller/command", 1000);
  ros::Publisher I_f_controller = n.advertise<std_msgs::Float64>("/I_f_controller/command", 1000);
  ros::Publisher M_f_controller = n.advertise<std_msgs::Float64>("/M_f_controller/command", 1000);
  ros::Publisher RL_f_controller = n.advertise<std_msgs::Float64>("/RL_f_controller/command", 1000);
  ros::Publisher Th_f_controller = n.advertise<std_msgs::Float64>("/Th_f_controller/command", 1000);
  ros::Publisher Th_a_controller = n.advertise<std_msgs::Float64>("/Th_a_controller/command", 1000);
  ros::Rate loop_rate(20);

  int count = 0;
  float i = 0.0;
  int fl = 0;
  float angs[8] = {-1.5,
		-1.5,
		-1.5,
		-1.5,
		-1.5,
		-1.5,
		-1.5,
		-1.5};
  float sensor = 0;
  float contact_sensor = 0;

  /* Initialization of sensors variables */
  float h_st, h_ct, r_st, r_ct;

  /* Initialization of controller signals */
  float c_signal = 0.0f;
  int fl_up = 0;
  float set_p = 0; // This is the target sensor value we want to achieve
  float kp = 0.002; // Proportional gain for controller

  std_msgs::String msg;
  std::stringstream ss;

  /* Initialization of variables for counting elapsed time */
  float curr_time = 0;
  ros::Time time_begin = ros::Time::now();
  ros::Duration duration;
  ros::Time time_current;

  /* Initialization of variables for data recording */
  float recorded_data[4] = { 0.0f, 0.0f, 0.0f, 0.0f }; // Array containing the data to record
  int n_p = 10; // Precision of the recorded data
  std::cout << std::setiosflags(std::ios::fixed); // Setting the desired precision for the array

  myfile.open("/home/diego/catkin_ws/src/test_talker_listener/src/robot_hand_callibration.csv"); // Name and path for the csv file containing the recorded data (Replace with your file location)
  myfile << "Time" << "," << "Control Signal" << "," << "Human_stretch_sensor" << "," << "Human_contact_sensor" << "," << "Robot_stretch_sensor" << "," << "Robot_contact_sensor" << "\n"; // Headers for the recorded data

  /* Messages for joint n of robot hand */
  std_msgs::Float64 msg16_0;
  std_msgs::Float64 msg16_1;
  std_msgs::Float64 msg16_2;
  std_msgs::Float64 msg16_3;
  std_msgs::Float64 msg16_4;
  std_msgs::Float64 msg16_5;
  std_msgs::Float64 msg16_6;
  std_msgs::Float64 msg16_7;


  while (ros::ok())
  {
    /* Reading the sensors data */
    h_st = h_stretch.data;
    h_ct = h_contact.data;
    r_st = r_stretch.data;
    r_ct = r_contact.data;

    sensor = h_stretch.data;
    contact_sensor = h_contact.data;

    time_current = ros::Time::now(); // Reading the current time from ROS
    duration = time_current - time_begin; // Calculating the duration in a ROS Time variable
    curr_time = duration.toSec(); // Assigning the elapsed time in seconds to a float variable

    //ss << " Current Time: " << curr_time << "   Control Signal: " << c_signal << "   Sensor Signal: " << r_st << "   Set Point: " <<  set_p << "\n";
    ss << " Current Time: " << curr_time << "   Control Signal: " << c_signal << "   Sensor Signal: " << r_st << "   Set Point: " <<  set_p <<  "   Angle Human: " <<  get_h_angle(h_st) <<   "    Sensor Human: " <<  h_st << "\n";
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    // Initial hand configuration
    angs[0] = 0.0;
    angs[1] = 0.0;
    angs[2] = 0.0;
    angs[4] = -3.0;
    angs[5] = -3.0;
    angs[6] = -2.0;
    angs[7] = -2.0;

    /* Recording Data */
    recorded_data[0] = curr_time;
    recorded_data[1] = c_signal;
    recorded_data[2] = h_st;
    recorded_data[3] = h_ct;
    recorded_data[4] = r_st;
    recorded_data[5] = r_ct;
    record_data(recorded_data, n_p);
    // ////////////// //

    /* Controller */
    set_p = get_target_sensor_value(get_h_angle(h_st));
    c_signal = c_signal - kp*(set_p - r_st); // Proportional Controller
    if(curr_time > 5000)
    {
        myfile.close();
        break;
    }
    angs[3] = c_signal; // Command for the PIP joint of the index finger

    /* Security check for control signal saturation (to avoid any damage to the hand) */
    if(c_signal < -3.0)
        c_signal = -3.0;
    else if(c_signal > 2.0)
        c_signal = 2.0;
    // ///////////////////////////////////////////////////////////////////////////// //

    /* Security check for control signal saturation (to avoid any damage to the hand) */
    if(angs[3] < -3.0)
        angs[3] = -3.0;
    else if(angs[3] > 2.0)
        angs[3] = 2.0;
    // ///////////////////////////////////////////////////////////////////////////// //

    /* Assigning the desired control signals to each message to be pusblished */
    msg16_0.data = angs[0];
    msg16_1.data = angs[1];
    msg16_2.data = angs[2];
    msg16_3.data = angs[3];
    msg16_4.data = angs[4];
    msg16_5.data = angs[5];
    msg16_6.data = angs[6];
    msg16_7.data = angs[7];

    /* Publishing messages to each topic corresponding to each joint */
    W_f_controller.publish(msg16_0);
    W_r_controller.publish(msg16_1);
    W_a_controller.publish(msg16_2);
    I_f_controller.publish(msg16_3);
    M_f_controller.publish(msg16_4);
    RL_f_controller.publish(msg16_5);
    Th_f_controller.publish(msg16_6);
    Th_a_controller.publish(msg16_7);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
