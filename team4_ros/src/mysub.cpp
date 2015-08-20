#include "ros/ros.h" 
#include <team4_ros/binIsFull.h> 

void mypubCallback(const team4_ros::binIsFull::ConstPtr& msg) 
{ 
	ROS_INFO("sub echoing pub: %d",msg->my_counter);
}

int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "b"); 

	// ROS comms access point 
	ros::NodeHandle my_handle;
        
        // tell master you want to sub to topic 
	ros::Subscriber mysub_object = my_handle.subscribe("ivy_topic",100,mypubCallback); 

	// act on callbacks 
	ros::spin();

	return 0; 
}
