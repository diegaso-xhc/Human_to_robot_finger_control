# Control of robot fingers via human hands

## Overview of the repostitory
<div align="justify">
The usage of robotic hands for grasping is one of the main disciplins being explored within the robotics field in recent years. State of the art robot hand designs and control algorithms have not yet reached a performance level comparable to human hands. Partly the reason for this is the lack of sensing both on a tactile level (interactions with the environment, e.g., forces, temperature, among others) and on a mechanical level. The constrained available space in robotic hands, makes embedding position and torque sensors unfeasible. 

<br />

In an attempt to provide the robotics community with an alternative for the aforementioned restrictions, the authors have proposed a general framework for tactile sensing. We showed that this <a href="https://ieeexplore.ieee.org/document/10161344">framework</a> can be used to control a robot hand with sensor measurements in a human hand. Please see the <a href="https://www.youtube.com/watch?v=i43wgx9bT-E">video</a> for reference. Additional media attention to this topic can be seen <a href="https://www.tum.de/en/news-and-events/all-news/press-releases/details/robotik-neue-hautaehnliche-sensoren-passen-fast-immer">here</a>.

<br />

This repository contains ROS based algorithms to control an RH8D Seed Robotics robot hand via tactile sensing signals. These signals can be effortlessly replaced with other sensor signals, making the control of this robot hand easily implementable.

<br />
<br />
Several high performance platforms (e.g. Matlab) offer a control systems toolbox which can be used reliably to model and control systems. Nevertheless, these toolboxes tend to be built on proprietary functions, which can't be customized depending on user needs. This repository is intended to provide an open source platform with similar functionalities for representing systems responses and understanding what controllers are suitable for them. Additionally, thanks to its versatility and structure, this repository can be used for teaching purposes, helping students understand the magic happening behing control toolboxes.
<br /> 
<br />
This toolbox was used on for the development of the <a href="https://www.sciencedirect.com/science/article/pii/S2212827117302305">scientific article</a>, which shows an approach to reduce inaccuracies on five-axis CNCs.
<br /> 
<br /> 
 <p align="center">
   <img src="/Visualizations/Time_response_cursor.png" width="700" />
</p>
<br />

## Understanding repository

The repository was developed in C#, using the following software version:

```
- Microsoft Visual Studio Community 2019 (Version 16.11.29)
- Target Framework: .NET Framework 4.6.1
- OxyPlot.Wpf (Version 2.1.2) --> Can be installed using NuGet package manager
- OxyPlot (Version 2.1.2) --> Can be installed using NuGet package manager
- OxyPlot.Wpf.Shared (Version 2.1.2) --> Can be installed using NuGet package manager
```

The project has been compiled using the aforementioned libraries into a self contained project. Nevertheless, if some compilation errors arise, please check the aforementioned versions. The most relevant files on the repository are detailed as follows:

```
- backupFunctions.cs --> Operates on parameters within system modeling, such as raw phase and angle vectors extracted from a system's response.
- DifferentialEquations.cs --> Handles system model representations, transfer functions arithmetics, contains Runge–Kutta methods, prints different model representations of a system.
- fastFourierTransform.cs --> Transforms time space vectors into the frequency (fast Fourier transform) and viceversa (inverse fast Fourier transform).
- Matrix.cs --> Handles matrices in an efficient manner.
- MainWindow.xaml --> Contains the code required by WPF to launch the GUI for the user.
- MainWindow.xaml.cs --> Handles user requests and returns required outputs.
```
<br />
UPDATE (09.2023): Due to a recent change of the chart visualization library, it is worth noticing at the moment only the time response and frequency spectrum can be displayed graphically. The authors are currently updating the Bode and Nichols diagrams. Nevertheless, besides visualization, all functions are implemented and fully functional. The user simply needs to print the outputs or create methods to use them.
<br />
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
