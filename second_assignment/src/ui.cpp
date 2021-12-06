#include "ros/ros.h"
#include "second_assignment/Velocity.h"
using namespace std;

ros::ServiceClient client;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "change_velocity_node");
	ros::NodeHandle nh;

	client = nh.serviceClient<second_assignment::Velocity>("/velocity");
    

    cout<<"Set a velocity between 1 and 10\n"<<endl;
    float val;

    second_assignment::Velocity vel_srv;
    while(1)
    {
        cin >> val;
        if(val < 1 || val > 10)
            cout<<"Input not valid, retry\n"<<endl;
        else if(val == 5)
            return 1;
        vel_srv.request.req_velocity = val;
        client.waitForExistence();
        client.call(vel_srv);
    }
   // ros::spin();
		return 0;
}