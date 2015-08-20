#include "ros/ros.h"
#include <stdlib.h>
#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <queue>
#include <sstream>
#include "math.h"
#include <unistd.h>
#include "std_msgs/String.h"

int x;
float z;
ros::Publisher bin_pub;
bool isSent=false;

void groundTruthCallback(const nav_msgs::Odometry msg) 
{     
    //Update Current Position
	geometry_msgs::Pose currentLocation;
    currentLocation = msg.pose.pose;
	
	geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = (-1.75) - currentLocation.position.x;
    directionVector.y = 32 - currentLocation.position.y;
    directionVector.z = 0;

	ROS_INFO("Bin x distance: [%f]", directionVector.x);
    ROS_INFO("Bin Y distance: [%f]", directionVector.y);


	
	//if((directionVector.x<0.2&&directionVector.x>-0.2)&&(directionVector.y<1&&directionVector.y>-1)){}
	if(!isSent){
	sleep(30);
	ROS_INFO("I have reached my destination!");
	std_msgs::String mypub_msg;
		mypub_msg.data = "I AM Full"; 
	  bin_pub.publish(mypub_msg);
	isSent=true;
	}
	
}





int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "bin_node");
	
	// Publisher
	ros::NodeHandle n;
	bin_pub = n.advertise<std_msgs::String>("bin_topic",100);
	ros::Publisher bin_pub = n.advertise<std_msgs::String>("bin_topic",100);
    
	// Subscriber    
	ros::NodeHandle sub_handle; 
	ros::Subscriber mysub_object;
	mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_3/base_pose_ground_truth",1000, groundTruthCallback); 

	// loop 10 Hz 
	ros::Rate loop_rate(10);

	while (ros::ok()) 
	{
	 

		ros::spinOnce();
		loop_rate.sleep();

	} 

	return 0; 
}

