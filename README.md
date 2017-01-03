# medbot
Mario here!! I had a blast during the last few months becoming familiar with ROS (Robot Operating System) and ultimately making Medbot. I decided to share my experiences and code so others in the community could build similar things.

Medbot is a low cost personal care robot aimed at assisting recovering patients with their medical equipment. It is a fully autonomous design that draws heavily from Turtlebot. Turtlebot’s launch chain was kept given how easy it is to reconfigure your robot depending on the particular base and sensors you use. An iRobot Create 2 base served as the foundations for Medbot and an Intel Realsense R200 as its main sensor for mapping and navigation, all orchestrated by an Intel Atom SBC. 

The driver for the base is a modified version of AutonomyLab’s create_autonomy C++ library. More precisely, it has a Nodelet version of ca_node with some additional features. Among them: PWM Control of the main brush motor provided through a subscriber, A safety controller for the base in case of bumper or cliff sensor events, A neat little GUI to visualize diagnostic information and control some aspects of the base, etc. Futures updates will include rewriting most subscribers in ca_node as services which in my opinion will conform more to ROS guidelines given what their callback functions are used for, which is requesting something through Create’s OpenInterface.

Medbot’s odometry was improved by means of a BNO055 IMU (node written in python) and the robot_localization package, which is a general-purpose state estimation package that can fuse an arbitrary number of sensor data. Other components of Medbot include a 5V breaker board node to control some other devices.

This is all for now. I will be updating the documentation soon with installation instructions and maybe create a wiki.
