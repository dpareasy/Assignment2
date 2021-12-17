#Research Track I - second assignment
================================
Parisi Davide Leo 4329668 

This assignment requires the development of a software architecture, in c++ language, using ROS operating system to constraints drive a robot around a particular environment. The software relies on sensor_msgs package to computing the distances of the robot from the wall and on geometry_msgs package for setting robot velocity around the circuit.
The architecture should rely on:

* A node for the control of the robot

* An additional node which interacts with the user to increase/ decrease velocity and resets the robot position

## Content of the package ##

* `src`: this is a folder containing all C++ executables used in this package

* `launch`: this is a folder containing the .launch extension file needed to launch the simulation

* `world`: this is a folder containing the enviroment in which the robot should move

* `srv`: this is a folder containing the service created for changing velocity

* `CMakeList.txt`: this is a file with information about the compilation

* `package.xml`: this is a file with information about the compilation

In particular, the folder `src` contains:

* `controller.cpp`: This is the principal node that controls robot behavior

* `ui.cpp`: this is the user interface node that allows the user to change the robots linear and angular velocity and also makes him able to reset the robot position and quit all the programs

## How to run ##

To run the script in the simulator, use simply a .launch script that executes the world, the controller node and the user interface node as follow:

```
$ roslaunch second_assignment run.launch
```
where `second_assignment` is the name of the package created for this assignment and `run.launch` is the launch script.
Whit this command, roscore is launched by default.

## Robot behavior ##

When the user launches the simulation, the robot is spawned in a pre-built environment waiting for input from the user to start moving. The robot is equipped with a laser scanner that allows to constantly control the distances from the walls in an angular range of 180 degrees. From the ui.cpp node, the user can change the velocity of the robot moving around the circuit. It can also reset the robot position, which will restart his run with the last velocity registered before the reset command. Here I imposed a minimum velocity equal to zero and a maximum one equal to 8. If the velocity is at its maximum value it is reasonable to see the robot crash into the wall when it is curving, while if the velocity is at a reasonable value, the robot never looses its control.

![Flowchart](https://user-images.githubusercontent.com/92155300/146471511-fd1c2e09-8511-4c41-a4b9-8dcdabb47073.jpg)


## About software architecture ##

This project is based on the use of three different scripts, the /world , the /controller.cpp, and the /ui.cpp. They communicate with each other with messages and services. In particular, the world node uses a message for publishing robot position within the environment in the `base_scan` topic, so that the /controller can read from it. The /controller instead, uses a service for publishing in `cmd_vel` topic the velocity of the robot, so that the world node can read from it. In addition to this, I defined a Service file in the srv directory, named `Velocity.srv`, used in ui.cpp to give the possibility to the user to increment or decrement the robot linear and angular velocity in the circuit. The ui.cpp uses also another service, the `Empty` type of service defined in `std_srvs` package, useful to reset the robot position within the enviroment.

## Pseudocode ##

### controller.cpp ###

```
define MAX_VELOCITY
define MIN_VELOCITY
initialize velocity 

if (Client send a request)

    if(request == 1 && velocity < MAX_VELOCITY)
        save value of incremented velocity
    else 
        velocity equal to max velocity 
    
    if(request == 2 && velocity > MIN_VELOCITY)
        save value of decremented velocity
    else 
        velocity equal to min velocity

    send response to user interface


Divide ranges array into five subarrays
array1 = right side
array3 = front
array5 = left side

compute minimum value for each of the three arrays

initial linear velocity equal to zero

if (front <= threashold)
    if(right side > left side)
        turn right
    if(left side > right side)
        turn left
```
### ui.cpp ###

```
while (1)
    ask the user to type a choice
    switch(choice)
        case 'a':
            send request to increase velocity
            print the server answer
            break;
        case 'b':
            send request to decrease velocity 
            print server answer
            break;
        case 'r':
            send request to reset robot position
            break;
        case 'q':
            quit all the programs
            brak;
        default:
            incorrect input
            ask again to type a correct choice
            break;
```
## Possible improvements ##

One possible improvement for this project could be to add other controls to make the robot able to move in the middle of the circuit. In fact, during its ride, the robot tends towards the roadside edge.

