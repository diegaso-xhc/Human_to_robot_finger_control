# Control of robot fingers via human hands

## Overview of the repostitory
<div align="justify">
The usage of robotic hands for grasping is one of the main disciplins being explored within the robotics field in recent years. State of the art robot hand designs and control algorithms have not yet reached a performance level comparable to human hands. Partly the reason for this is the lack of sensing both on a tactile level (interactions with the environment, e.g., forces, temperature, among others) and on a mechanical level. The constrained available space in robotic hands, makes embedding position and torque sensors unfeasible. 
<br />
<br />
In an attempt to provide the robotics community with an alternative for the aforementioned restrictions, the authors have proposed a general framework for tactile sensing. We showed that this <a href="https://ieeexplore.ieee.org/document/10161344">framework</a> can be used to control a robot hand with sensor measurements in a human hand. Please see the <a href="https://www.youtube.com/watch?v=i43wgx9bT-E">video</a> for reference. Additional media attention to this topic can be seen <a href="https://www.tum.de/en/news-and-events/all-news/press-releases/details/robotik-neue-hautaehnliche-sensoren-passen-fast-immer">here</a>.
<br />
<br />
This repository contains ROS based algorithms to control an RH8D Seed Robotics robot hand via tactile sensing signals. These signals can be effortlessly replaced with other sensor signals, making the control of this robot hand easily implementable.
<br />
<br />
<p align="center">
   <img src="/Visualizations/Human_to_robot_hand.png" width="700" />
</p>
<br />

## Understanding repository

The repository was developed using the following software version:

```
- Ubuntu 18.04
- ROS Melodic
```

The project has been tested using the aforementioned software versions. However, it can be effortlessly extended to newer Ubuntu or ROS versions. This repository contains the following:

```
- dynamixel motor --> A collection of ROS packages which deals with the low level control of dynamixel motors (used in the RH8D hand).
- my_dynamixel_tutorial --> A Package containing the required nodes to launch the RH8D hand within Ubuntu 18.04.
- sensor_pkg --> A package developed by Seed Robotics for reading signals from the robot hand's fingertip tactile sensors.
- Sensors_read --> A python package developed to reading serial signals (up to 16 at the time) which can be used to control the hand directly.
- RH8D_control --> A c++ package developed to subscribe to the sensors topic and map signals to robot hand motions. 
```
<br />

## Contributions

The contributions of this repository can be summarized as follows:

```
- Algorithms for handling differential equations, transfer functions, and state space representations
- Algorithms to manipulate matrices in an intuitive and efficient manner
- Algorithms to operate on system response parameters (e.g. angle, phase, etc.)
- An intuitive GUI (with similar nomenclature as MATLAB for transfer functions) to analyze systems time and frequency response
- An open source code, which can be used for teaching of fundamentals of control systems.
```

## Examples of GUI usage

### Time response to a Sine-wave-like excitement signal

The following figure shows the time response of a system to a sine wave excitement signal of 4Hz. Note that in order to obtain the time response, the user needs to do the following:

```
- Input the transfer function: [numerator separated by commas];[denominator separated by commas]
- Click on Create Model
- Input the sampling time (dt), initial time (t(t0)), and final time in seconds.
- If the user requires a step response, click on Step response. For sine wave excitements, please input the frequency of the sine wave and click on Sine response.
- The user can then click on get time response or view frequency spectrum.
- Although visualizations for Bode and Nichols charts are not available at the moment, the user can still click them and extract the frequency responses from the library.
```

<p align="center">
   <img src="/Visualizations/Sine_response.PNG" width="650" />
</p>

### Time response to a Step-like excitement signal

The following figure shows the time response of a system to a step-like excitement signal:

<p align="center">
  <img src="/Visualizations/Systems_time_response.PNG" width="650" />  
</p>

### Frequency spectrum of the input signal

The following figure shows the frequency spectrum of an input sine-wave-like signal of 52Hz:

<p align="center">
   <img src="/Visualizations/Frequency_spectrum.PNG" width="650" />
</p>

## License

Developed by Diego Hidalgo C. (2021). This repository is intended for research purposes only. If you wish to use any parts of the provided code for commercial purposes, please contact the author at hidalgocdiego@gmail.com.
