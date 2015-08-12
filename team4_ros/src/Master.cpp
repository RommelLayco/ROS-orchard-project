#include "ros/ros.h" 
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h" 
#include "geometry_msgs/Twist.h"

ros::Publisher move_pub; 
int x;
float z;

void binCallback(const std_msgs::String msg)
{
if (msg.data.compare("I AM Full") != 0 )

}




int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "master_node");
	
	// ROS comms access point 
	ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("bin_topic", 1000, binCallback);

    // master registry pub/sub 

	// loop 10 Hz 
	ros::Rate loop_rate(10);
	
	x = 1;
	z = 0;
	
    int counter=0;
	while (ros::ok()) 
	{

		ros::spinOnce();
		loop_rate.sleep();

	} 

	return 0; 
}

