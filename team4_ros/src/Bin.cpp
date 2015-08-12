#include "ros/ros.h" 
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h" 
#include "geometry_msgs/Twist.h"
#include "team4_ros/binIsFull.h"

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
	move_pub = n.advertise<team4_ros::binIsFull>("binIsFull_topic",100);

	// loop 10 Hz 
	ros::Rate loop_rate(10);
	
	x = 1;
	z = 0;
	
    int counter=0;
	while (ros::ok()) 
	{
        team4_ros::binIsFull bin_msg;
        bin_msg.isBinFull=false;
	    move_pub.publish(bin_msg);
	    z = 0;
		ros::spinOnce();
		loop_rate.sleep();

	} 

	return 0; 
}

