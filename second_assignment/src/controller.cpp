#include "ros/ros.h"
#include "geometry_msgs/Twist.h"//we are going to subscribe 
#include "sensor_msgs/LaserScan.h"

//defining the dimension of each array for visual ranges
#define RIGHT_DIM 40
#define FRONT_RIGHT_DIM 240
#define FRONT_DIM 160
#define FRONT_LEFT_DIM 240
#define LEFT_DIM 40

//Here, as global variable are defined the array for visual ranges
float right_dist[RIGHT_DIM];
float front_right_dist[FRONT_RIGHT_DIM];
float front_dist[FRONT_DIM];
float front_left_dist[FRONT_LEFT_DIM];
float left_dist[LEFT_DIM];

//Trheshold distance  from the wall setted to 1. 
//This is the maximum distance from which the robot can approaches
float dist_min = 1;
float default_velocity = 1.0;

//The publisher is defined as global variable because 
//I have to initialize it in the Main function but 
//use it in the Callback
ros::Publisher pub; 

void Callback(const sensor_msgs::LaserScan::ConstPtr& msg)//we have a pointer of type sensor_msgs::LaserScan---the specific message that I'm going to receive
{
	//Here each array for visual ranges is created. I decided to divide the visual field of the robot in 5 sections
	//but for make it avoid distances front_right_distance and front_left_distance is not used.
	for (int i = 0; i < msg->ranges.size(); i++) 
	{
		//The array for the right visual field. It is between an agle of 170 and 180 degrees.
		for(int h = 0; h < RIGHT_DIM; h++)
		{
			right_dist[h] = msg->ranges[i++];
		}
		//The array for the front-right visual field. It is between an agle of 110 and 170 degrees.
		for(int j = 0; j < FRONT_RIGHT_DIM; j++)
		{
			front_right_dist[j] = msg->ranges[i++];
		}
		//The array for the frontal visual field. It is between an agle of 70 and 110 degrees.
		for(int k = 0; k < FRONT_DIM; k++)
		{
			front_dist[k] = msg->ranges[i++];
		}
		//The array for the front-left visual field. It is between an agle of 10 and 70 degrees.
		for(int l = 0; l < FRONT_LEFT_DIM; l++)
		{
			front_left_dist[l] = msg->ranges[i++];
		}
		//The array for the left visual field. It is between an agle of 0 and 10 degrees.
		for(int n = 0; n < LEFT_DIM; n++)
		{
			left_dist[n] = msg->ranges[i++];
		}
	}
		//To know the direction in which to move the robot,
		//it is required to know the minimum distance from the wall in each field.
		//To do this it is simply needed to check the minimum value for each array.
		float min_right = right_dist[0];
		for(int i=0; i<RIGHT_DIM; i++)
		{
			if(min_right > right_dist[i])
				min_right = right_dist[i];
		}
		float min_front_right = front_right_dist[0];
		for(int i=0; i<FRONT_RIGHT_DIM; i++)
		{
			if(min_front_right > front_right_dist[i])
				min_front_right = front_right_dist[i];
		}
		float min_front = front_dist[0];
		for(int i=0; i<FRONT_DIM; i++)
		{
			if(min_front > front_dist[i])
				min_front = front_dist[i];
		}
		float min_front_left = front_left_dist[0];
		for(int i=0; i<FRONT_LEFT_DIM; i++)
		{
			if(min_front_left > front_left_dist[i])
				min_front_left = front_left_dist[i];
		}
		float min_left = left_dist[0];
		for(int i=0; i<LEFT_DIM; i++)
		{
			if(min_left > left_dist[i])
				min_left = left_dist[i];
		}
	geometry_msgs::Twist my_vel;

	//
	my_vel.linear.x = 5*default_velocity;

	if (min_front < dist_min)
	{
		if(min_right > min_left)
		{
			my_vel.linear.x = 0.3*default_velocity;
			my_vel.angular.z = -2.5*default_velocity;
		}
		if(min_right < min_left)
		{
			my_vel.linear.x = 0.3*default_velocity;
			my_vel.angular.z = 2.5*default_velocity;
		}
	}
	pub.publish(my_vel);
	
}


int main(int argc, char **argv)
{
	//initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "robot_node");
	ros::NodeHandle nh;
	//define the subscriber to compute the distance from walls
	ros::Subscriber sub = nh.subscribe("/base_scan", 1, Callback);//the topic in which the other nodes use to publish their position 
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::spin();

		return 0;
}
