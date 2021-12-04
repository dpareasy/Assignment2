#include "ros/ros.h"
#include "geometry_msgs/Twist.h"//we are going to subscribe 
#include "sensor_msgs/LaserScan.h"

#define RIGHT_DIM 40
#define FRONT_RIGHT_DIM 260
#define FRONT_DIM 120
#define FRONT_LEFT_DIM 260
#define LEFT_DIM 40

float right_dist[RIGHT_DIM];
float front_right_dist[FRONT_RIGHT_DIM];
float front_dist[FRONT_DIM];
float front_left_dist[FRONT_LEFT_DIM];
float left_dist[LEFT_DIM];

float dist_min = 1;

//we need to write the callback
ros::Publisher pub; //used as global variable because I have to define it in the Main function but used in the Callback

void Callback(const sensor_msgs::LaserScan::ConstPtr& msg)//we have a pointer of type sensor_msgs::LaserScan---the specific message that I'm going to receive
{
	//creo i vettori distanza a destra, distanza a sinistra e distanza frontale	
	for (int i = 0; i < msg->ranges.size(); i++) 
	{

		for(int h = 0; h < RIGHT_DIM; h++)
		{
			right_dist[h] = msg->ranges[i++];
			//ROS_INFO("right distances@[%f, %d]", right_dist[j], j);
		}
	
		for(int j = 0; j < FRONT_RIGHT_DIM; j++)
		{
			front_right_dist[j] = msg->ranges[i++];
			//ROS_INFO("right distances@[%f, %d]", right_dist[j], j);
		}
		for(int k = 0; k < FRONT_DIM; k++)
		{
			front_dist[k] = msg->ranges[i++];
			//ROS_INFO("front distances@[%f, %d]", front_dist[k], k);
		}
		for(int l = 0; l < FRONT_LEFT_DIM; l++)
		{
			front_left_dist[l] = msg->ranges[i++];
			//ROS_INFO("left distances@[%f, %d]", left_dist[l], l);
		}
		for(int n = 0; n < LEFT_DIM; n++)
		{
			left_dist[n] = msg->ranges[i++];
			//ROS_INFO("left distances@[%f, %d]", left_dist[l], l);
		}
	}

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

	//if(right_dist)
	my_vel.linear.x = 2.0;

	if (min_front < dist_min)
	{
		if(min_right > min_left)
		{
			my_vel.linear.x = 0.4;
			my_vel.angular.z = -0.8;
		}
		if(min_right < min_left)
		{
			my_vel.linear.x = 0.4;
			my_vel.angular.z = 0.8;
		}
	}
	

	//ROS_INFO("Controller@[%f, %d]", distances[i], i);
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
