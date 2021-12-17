#Research Track I - second assignment
================================
Parisi Davide Leo 4329668 

This assignment requires the development of a software architecture, in c++ language, using ROS operating system, to constraints drive a robot around a particular environment. The software relies on type message sensor_msgs/LaserScan for computing the distances of the robot from the wall and on type message geometry_msgs/Twist for setting robot velocity around the circuit.
The architecture should rely on:

* A node for the control of the robot

* An additional node which interacts with the user to increase/ decrease velocity and resets the robot position

## Content of the package ##

* `src`: this is a folder containing all C++ executables used in this package

* `launch`: this is a folder containing the .launch extension file needed to launch the simulation

* `world`: this is a folder containing the world in which the robot should move

* `srv`: this is a folder containing the service created for changing velocity

* `CMakeList.txt`: this is a file with information about the compilation

* `package.xml`: this is a file with information about the compilation

In particular, the folder `src` contains:

* `controller.cpp`: This is the principal node that controls robot behavior

* `ui.cpp`: this is the user interface node that allows the user to change the robots linear and angular velocity and also makes him able to reset the robot position and quit all the programs

## How to run ##

To run the script in the simulator, use simply a .launch script that executes the world, the controller node, and the user interface node as follow:

```
$ roslaunch second_assignment run.launch
```
where `second_assignment` is the name of the package created for this assignment and `run.launch` is the launch script.
Whit this command roscore is launched by default.

## Robot behavior ##

When the user launches the simulation, the robot is spawned in a pre-built environment waiting for input from the user to start moving. The robot is equipped with a laser scanner that allows to constantly control the distances from the walls in an angular range of 180 degrees. From the ui.cpp node, the user can change the velocity of the robot moving around the circuit. It can also reset the robot position, which will restart his run with the last velocity registered before the reset command. Here I imposed a minimum velocity equal to zero and a maximum one equal to 8. If the velocity is at its maximum value it is reasonable to see the robot crash into the wall when it is on the narrowst curve, while if the velocity is at a reasonable value, the robot never looses its control.

![Flowchart](https://user-images.githubusercontent.com/92155300/146471511-fd1c2e09-8511-4c41-a4b9-8dcdabb47073.jpg)


## About software architecture ##

This assignment bases on the use of three different scripts, the /world , the /controller.cpp, and the /ui.cpp. They communicate with each other with messages and services. In particular, the world node uses a message for publishing robot position within the environment in the /base_scan topic, so that the /controller can read from it. The /controller instead, uses a service for publishing in /cmd_vel topic the velocity of the robot, so that the world node can read from it. In addition to this, I defined a Service file in the srv directory used to give the possibility to the user to increment or decrement the robot linear and angular velocity in the circuit. The /ui.cpp takes commands as input from the user, then it handles these commands with a switch() and sends a request to the /controller.cpp, which replies with changing the velocities.

