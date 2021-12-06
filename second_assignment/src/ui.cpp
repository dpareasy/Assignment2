#include "ros/ros.h"
#include "second_assignment/Velocity.h"
using namespace std;

ros::ServiceClient client;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "change_velocity_node");
	ros::NodeHandle nh;

	client = nh.serviceClient<second_assignment::Velocity>("/velocity");
    

    cout << "Velocity controller, press:\n a) if you want to increase velocity\n b) if you want to decrease velocity\n q)if you want to quit the program " << endl;
    float val;
    char choice;

    second_assignment::Velocity vel_srv;
    
    while(1)
    {
        
        cin >> choice;
        switch(choice)
        {
            case 'A':
            case 'a':
                cout << "increasing velocity\n"<<endl;
                val = 1.0;
                vel_srv.request.req_velocity = val;
                client.waitForExistence();
                client.call(vel_srv);
                break;
            case 'B':
            case 'b':
                cout<<"decreasing velocity\n"<<endl;
                val = -1.0;
                vel_srv.request.req_velocity = val;
                client.waitForExistence();
                client.call(vel_srv);
                break;
            case 'Q':
            case 'q':
                cout << "Program exiting...\n"<<endl;
                return 1;
                break;
            default:
                cout<<"Invalid input, retry\n"<<endl;
                break;        

        }
        
    }
    return 0;
}