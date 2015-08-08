#include "ros/ros.h"
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

double normalizeAngle(double angle)
{
    while (angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
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

void updateCurrentVelocity(){

// Find the correct angle
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation

    geometry_msgs::Point desiredLocation;
    desiredLocation.x = 1;
    desiredLocation.y = 1;	
    desiredLocation.z = 0;

    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;
    
    // Thank god we're only doing 2D stuff
    double desiredAngle = atan2(directionVector.y, directionVector.x);

        if (currentAngle != desiredAngle) {
        // Turn towards angle
        currentVelocity.linear.x = 0;
        
        if (turnAnticlockwise(currentAngle, desiredAngle)) {
            // Turn anti clockwise
            currentVelocity.angular.z = 1;
        } else {
            // Turn clockwise
            currentVelocity.angular.z = -1;
        }
    } else {
        // Go forward
        currentVelocity.linear.x = 1;
        currentVelocity.angular.z = 0;
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
	ros::Rate loop_rate(1000);

	mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000, stageOdometryCallback); 



	        currentVelocity.linear.x = 1; 
		currentVelocity.linear.y = 0; 
                currentVelocity.angular.z = 0.00;
	


	while (ros::ok()) 
	{ 
		loop_rate.sleep();


		// refer to advertise msg type 

		mypub_object.publish(currentVelocity); 
		updateCurrentVelocity(); 
	} 

	return 0; 
}
