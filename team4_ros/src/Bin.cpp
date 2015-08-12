#include "ros/ros.h" 
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h" 
#include "geometry_msgs/Twist.h"



ros::Publisher move_pub;
int x;
float z;




int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "bin_node");
	
	// ROS comms access point 
	ros::NodeHandle n;


    // master registry pub/sub 
	ros::Publisher bin_pub = n.advertise<std_msgs::String>("bin_topic",100);
        
	// loop 10 Hz 
	ros::Rate loop_rate(10);
	
	
    int counter=0;
	while (ros::ok()) 
	{
		std_msgs::String mypub_msg;
		mypub_msg.data = "I AM Full";  
	   // bin_pub.publish(mypub_msg);
		ros::spinOnce();
		loop_rate.sleep();

	} 

	return 0; 
}

