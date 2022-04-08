#include <stdlib.h>
#include "ros/ros.h"
#include "second_assignment/Velocity.h"
#include "std_srvs/Empty.h"
using namespace std;

//declaring clients for change_velocity request
//and for reset position request
ros::ServiceClient client1;
ros::ServiceClient client2;

int main(int argc, char **argv)
{
    //initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "change_velocity_node");
	ros::NodeHandle nh;

    //defining clients for change_velocity request
    //and for reset position request
	client1 = nh.serviceClient<second_assignment::Velocity>("/velocity");
    client2 = nh.serviceClient<std_srvs::Empty>("/reset_positions");
    //initializing variables: one for value to pass to the server
    //and the other one for case handling 
    float val;
    char choice;

    //declaration of variables for messages
    second_assignment::Velocity vel_srv;
    std_srvs::Empty res_pos;

    //while loop for constantly asking the user to insert a command 
    while(1)
    {
        cout << "\nRobot controller, press:" <<endl;
        cout << "a) if you want to increase velocity " <<endl;
        cout << "d) if you want to decrease velocity " <<endl;
        cout << "r) if you want to reset the position " <<endl;
        cout << "q) if you want to quit the program " <<endl;
        cout << "command chosen: ";
        //getting the user choice from the keyboard
        cin >> choice;
        //choice handling:
        //- choose A/a to increase velocity 
        //- choose D/d to decrease velocity 
        //- choose R/r to reset robot position
        //- choose Q/q to quit the program
        switch(choice)
        {
            case 'A':
            case 'a':
            	system("clear");
                val = 1.0;
                //saving the value to send to the server in the request 
                vel_srv.request.req_change_velocity = val;
                //waiting for service existence
                client1.waitForExistence();
                //sending a request 
                client1.call(vel_srv);
                //if velocity has been increased print current velocity 
                if (vel_srv.response.succeded)
                {
                    cout<<"Velocity has been increased: "<<endl;
                    cout<<"linear velocity: "<<vel_srv.response.resp_change_velocityx<< "  " <<"angular velocity: "<<vel_srv.response.resp_change_velocityz<<endl;
                }
                //else inform the user that it cannot increment velocity
                else
                {
                    cout<<"Maximum velocity reached"<<endl;
                }
                break;
            case 'D':
            case 'd':
            	system("clear");
                //the same as for previous case
                val = 2.0;
                vel_srv.request.req_change_velocity = val;
                client1.waitForExistence();
                client1.call(vel_srv);
                //if velocity has been decreased print current velocity 
                if (vel_srv.response.succeded)
                {
                    cout<<"Velocity has been decreased: "<<endl;
                    cout<<"linear velocity: "<<vel_srv.response.resp_change_velocityx<< "  " <<"angular velocity: "<<vel_srv.response.resp_change_velocityz<<endl;
                }
                //else inform the user that it cannot decrement velocity
                else
                {
                    cout<<"Minimum velocity reached"<<endl;
                }
                break;
            case 'R':
            case 'r':
            	system("clear");
                //request for resetting position
                cout << "\nThe position has been resetted"<<endl;
                client2.waitForExistence();
                client2.call(res_pos);
                break;
            case 'Q':
            case 'q':
                //command for quit the program
                cout << "\nProgram exiting..."<<endl;
                return 1;
                break;
            default:
                cout<<"\nInvalid input, retry\n"<<endl;
                break;
        }
    }
    return 0;
}
