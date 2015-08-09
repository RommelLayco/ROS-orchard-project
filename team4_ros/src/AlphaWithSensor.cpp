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

// Current velocity of the Robot
geometry_msgs::Twist currentVelocity;

// Current location of the robot
geometry_msgs::Pose currentLocation;

// The current angle of the robot
double currentAngle;

// Speed and angular velocity whensensor detect something
int x;
float z;

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    ROS_INFO("Sensor:");
    for (i; i < 180; i++) {
         if (msg->ranges[i] < 1.5)
         {
            ROS_INFO("I'm near something! [%f]", msg->ranges[i]);
            if (i < 45)
            {
                z = 90.0;
            } else if (i >= 45 && i < 90)
            {
                z = 90;
            } else
            {
                z = -90;
            }
            
            
         }
    }
}



void updateCurrentVelocity(){

    // Find the correct angle


    geometry_msgs::Point desiredLocation;
    desiredLocation.x = -10;
    desiredLocation.y = -21;	
    desiredLocation.z = 0;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;
    
    // Calculate the desired angle
    double desiredAngle = atan2(directionVector.y, directionVector.x);

   if (z != 0){
	currentVelocity.linear.x = 2;
        currentVelocity.angular.z = z;
	return;

}

// If the desired angle is 0
if(desiredAngle == 0 || desiredAngle == M_PI)
{}
else
{    
	   
    //ROS_INFO("currentAngle is : %f",currentAngle); 
    //ROS_INFO("desiredAngle is : %f",desiredAngle); 

    // If the deifference between current angle and desired angle is less than 0.1 stop spining
    if (currentAngle-desiredAngle>0.1 || desiredAngle-currentAngle>0.1)
    {
        // Spin
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0.5;
    } else {
        // Go forward
        currentVelocity.linear.x = 1;
        currentVelocity.angular.z = 0;
    }
}
}



void groundTruthCallback(const nav_msgs::Odometry msg) 
{     


    //Update Current Position
    currentLocation = msg.pose.pose;
    double x = currentLocation.orientation.x;
    double y = currentLocation.orientation.y;
    double z = currentLocation.orientation.z;
    double w = currentLocation.orientation.w;
    
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
    currentAngle = yaw;
    //ROS_INFO("Yaw is : %f",yaw); 
	
}





int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "AlphaWithSensorRobotNode"); 

	// ROS node hander
	ros::NodeHandle velPub_handle;
	ros::NodeHandle sub_handle; 
	

	// master registry pub and sub
	ros::Publisher mypub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
	ros::Subscriber mysub_object;

	
	// loop 25 
	ros::Rate loop_rate(10);

	mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000, groundTruthCallback); 
	
	// ROS comms access point 
	ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("robot_0/base_scan", 1000, sensorCallback);



	while (ros::ok()) 
	{ 
		loop_rate.sleep();

		updateCurrentVelocity(); 
		// refer to advertise msg type 
		

		mypub_object.publish(currentVelocity); 
		z=0;

		ros::spinOnce();
		loop_rate.sleep();
	} 

	return 0; 
}
