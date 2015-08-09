//The swing robot is supposed to do swing action when it reaches at a certain point
//todo set a path for the robots
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

double currentAngle;
geometry_msgs::Point desiredLocation1;


geometry_msgs::Point desiredLocation2;



double normalizeAngle(double angle)
{
    while (angle < 0) {
        angle += M_PI;
    }
    while (angle > M_PI) {
        angle -= M_PI;
    }
    return angle;
}


bool turnAnticlockwise(double currentAngle, double desiredAngle)
{   
    if (currentAngle < 0) {
        currentAngle = 2 * M_PI + currentAngle;
    }
    if (desiredAngle < 0) {
        desiredAngle = 2 * M_PI + desiredAngle;
    }
    desiredAngle = normalizeAngle(desiredAngle - currentAngle);
    return desiredAngle < M_PI;
    
}

void swing(){
    //bool
}

void updateCurrentVelocity(){

// Find the correct angle
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    geometry_msgs::Point desiredLocation;
    desiredLocation1.x = 0;
desiredLocation1.y = 10; 
desiredLocation1.z = 0;
desiredLocation2.x = 0;
desiredLocation2.y = 10; 
desiredLocation2.z = 0;

    bool where=true;
    if (where){
        desiredLocation=desiredLocation1;
    }else{
        desiredLocation=desiredLocation2;
    }
    

    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;
    
    // Thank god we're only doing 2D stuff
    double desiredAngle = atan2(directionVector.y, directionVector.x);
    ROS_INFO("x is : %f",currentLocation.position.x); 
    ROS_INFO("y is : %f",currentLocation.position.y);
  
    
	if(desiredAngle == 0)
	{
                if(directionVector.x < 1 && directionVector.y<1){
        currentVelocity.linear.x = 0;
currentVelocity.angular.z = -0.5;
currentVelocity.angular.z = 0.5;
   where=!where;
        
  }


} else
{       
   // ROS_INFO("currentAngle is : %f",currentAngle); 
    //ROS_INFO("desiredAngle is : %f",desiredAngle); 
    

    if (currentAngle-desiredAngle>0.1 || desiredAngle-currentAngle>0.1)
    {
        // Turn towards angle
        currentVelocity.linear.x = 0;
        
        //if (turnAnticlockwise(currentAngle, desiredAngle)) {
            // Turn anti clockwise
            currentVelocity.angular.z = 0.5;
        //} else {
            // Turn clockwise
       
        //}
    } else {
        // Go forward
        currentVelocity.linear.x = 1;
        currentVelocity.angular.z = 0;
    }
}
}



void stageOdometryCallback(const nav_msgs::Odometry msg) 
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
    ROS_INFO("Yaw is : %f",yaw); 
	
}

int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "AlphaRobotNode"); 

	// ROS comms access point 
	ros::NodeHandle velPub_handle;
	ros::NodeHandle sub_handle;  

	// master registry pub 
	ros::Publisher mypub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
	ros::Subscriber mysub_object;
	// loop 10 Hz 
	ros::Rate loop_rate(25);

	mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000, stageOdometryCallback); 

	


	while (ros::ok()) 
	{ 
		loop_rate.sleep();

		updateCurrentVelocity(); 
		// refer to advertise msg type 

		mypub_object.publish(currentVelocity); 


		ros::spinOnce();
		loop_rate.sleep();
	} 

	return 0; 
}
