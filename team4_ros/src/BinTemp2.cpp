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
#include <team4_ros/binIsFull.h> 

int x;
float z;
ros::Publisher bin_pub;

// Current velocity of the Robot
geometry_msgs::Twist currentVelocity;

// Current location of the robot
geometry_msgs::Pose currentLocation;

void groundTruthCallback(const nav_msgs::Odometry msg) 
{     
    //Update Current Position
	//geometry_msgs::Pose currentLocation;
    currentLocation = msg.pose.pose;
	
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = (-1.75) - currentLocation.position.x;
    directionVector.y = 31 - currentLocation.position.y;
    directionVector.z = 0;

	//ROS_INFO("Bin x distance: [%f]", directionVector.x);
   // ROS_INFO("Bin Y distance: [%f]", directionVector.y);

	if(directionVector.y<0.3 && directionVector.y>-0.3){
                team4_ros::binIsFull mypub_msg; 
		mypub_msg.isFull = true; 
                mypub_msg.x= currentLocation.position.x;
                mypub_msg.y= currentLocation.position.y;
		bin_pub.publish(mypub_msg); 
	}
	
}

void exchange()
{
        currentVelocity.linear.x = 1;
        currentVelocity.angular.z = 2;	
}

int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "bin_node_t2");
	
	// ROS comms access point 
	ros::NodeHandle n;
	// master registry pub/sub 
	bin_pub = n.advertise<team4_ros::binIsFull>("bin_topic_t2",10);
  

    // master registry pub/sub 
	//bin_pub = n.advertise<std_msgs::String>("bin_topic",100);
        
	ros::NodeHandle sub_handle; 
	ros::Subscriber mysub_object;
	//mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_4/base_pose_ground_truth",1000, groundTruthCallback); 

	ros::NodeHandle velPub_handle;
	ros::Publisher binVelPub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_4/cmd_vel",1000);

	// loop 10 Hz 
	ros::Rate loop_rate(10);
	
	
    int counter=0;
	while (ros::ok()) 
	{
		exchange();
		ros::spinOnce();
		binVelPub_object.publish(currentVelocity);
		loop_rate.sleep();

	} 

	return 0; 
}

