#include "ros/ros.h"
#include <stdlib.h>
#include "std_msgs/String.h"
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

bool checkLocation(){
if (currentLocation.position.x==10 && currentLocation.position.y==21){

	return true;
}else{

	return false;
}


}

void updateCurrentVelocity(){

    // Find the correct angle


    geometry_msgs::Point desiredLocation;
    desiredLocation.x = 10;
    desiredLocation.y = 21;	
    desiredLocation.z = 0;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;
    
    // Calculate the desired angle
    double desiredAngle = atan2(directionVector.y, directionVector.x);

// If the desired angle is 0
if(desiredAngle == 0 || desiredAngle == M_PI)
{}
else
{    
	   
    //ROS_INFO("currentAngle is : %f",currentAngle); 
    //ROS_INFO("desiredAngle is : %f",desiredAngle); 

    // If the difference between current angle and desired angle is less than 0.1 stop spining
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
	ros::init(argc, argv, "DogNode"); 

	// ROS node hander
	ros::NodeHandle n;
	ros::NodeHandle sub_handle;
	ros::NodeHandle a;

	//ros::Subscriber barkSub = n.subscribe<>();

	// master registry pub and sub
	ros::Publisher mypub_object = n.advertise<std_msgs::String>("dog_topic",1000);
	ros::Publisher mypub_object2 = a.advertise<geometry_msgs::Pose>("robot_1/cmd_vel", 1000);
	ros::Subscriber mysub_object;

	// loop 25 
	ros::Rate loop_rate(10);

	mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_1/base_pose_ground_truth",1000, groundTruthCallback); 

	


	while (ros::ok()) 
	{ 
		loop_rate.sleep();

		updateCurrentVelocity();
		
		std_msgs::String mypub_msg;
		mypub_msg.data = "I AM BARKING!!Woof Woof!!";

		//mypub_object2.data = (currentLocation.position.x)

		mypub_object2.publish(currentLocation);


		if(checkLocation){
			mypub_object.publish(mypub_msg);
		}

		ros::spinOnce();
		

	} 

	return 0; 
}
