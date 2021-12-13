#include <stdlib.h>
#include "ros/ros.h"
#include "second_assignment/Velocity.h"
#include "std_srvs/Empty.h"
using namespace std;

ros::ServiceClient client1;
ros::ServiceClient client2;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "change_velocity_node");
	ros::NodeHandle nh;

	client1 = nh.serviceClient<second_assignment::Velocity>("/velocity");
    client2 = nh.serviceClient<std_srvs::Empty>("/reset_positions");
    

   
    float val;
    char choice;

    second_assignment::Velocity vel_srv;
    std_srvs::Empty res_pos;

  
    while(1)
    {
        cout << "\nRobot controller, press:" <<endl;
        cout << "d) if you want to increase velocity " <<endl;
        cout << "a) if you want to decrease velocity" <<endl;
        cout << "r) if you want to reset the position " <<endl;
        cout << "q) if you want to quit the program " <<endl;
        cout << "command chosen: ";
        cin >> choice;
        switch(choice)
        {
            case 'D':
            case 'd':
                cout << "\nVelocity has been increased"<<endl;
                val = 1.0;
                vel_srv.request.req_velocity = val;
                client1.waitForExistence();
                client1.call(vel_srv);
                break;
            case 'A':
            case 'a':
                cout<<"\nVelocity has been decreased"<<endl;
                val = -1.0;
                vel_srv.request.req_velocity = val;
                client1.waitForExistence();
                client1.call(vel_srv);
                break;
            case 'R':
            case 'r':
                cout << "\nThe position has been resetted"<<endl;
                client2.waitForExistence();
                client2.call(res_pos);
                break;
            case 'Q':
            case 'q':
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