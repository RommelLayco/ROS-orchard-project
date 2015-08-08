#include "ros/ros.h" 
#include "std_msgs/String.h" 
#include "geometry_msgs/Twist.h"

int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "AlphaRobotNode"); 

	// ROS comms access point 
	ros::NodeHandle my_handle; 

	// master registry pub 
	ros::Publisher mypub_object = my_handle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",100);

	// loop 10 Hz 
	ros::Rate loop_rate(10); 

	while (ros::ok()) 
	{ 
		loop_rate.sleep(); 

		// refer to advertise msg type 
                geometry_msgs::Twist mypub_msg; // replace existing similar assignments
                mypub_msg.linear.x = 1; 
		mypub_msg.linear.y = 0; 
                mypub_msg.angular.z = 0.00;
		mypub_object.publish(mypub_msg); 
	} 

	return 0; 
}
