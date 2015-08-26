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
#include <team4_ros/readyToExchange.h>
#include <team4_ros/arrived.h>

int x;
float z;
ros::Publisher exchange_object;
ros::Publisher binVelPub_object;
team4_ros::readyToExchange exchange_msg; 

// The current angle of the robot
double currentAngle;
double desiredAngle = 0;

bool readyToEx=false;
bool opreadyToEx=false;


// counter
int timeCount = 0; //for rotateAngle function

// Current velocity of the Robot
geometry_msgs::Twist currentVelocity;

// Current location of the robot
geometry_msgs::Pose currentLocation;

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
	
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = (-1.75) - currentLocation.position.x;
    directionVector.y = 31 - currentLocation.position.y;
    directionVector.z = 0;
    currentAngle = yaw + 3.14;
	
}

void rotateAngle(double angle2Turn, int angularSpd)
{
   //Calculate the angle to rotate  
    currentVelocity.linear.x = 0;
	ros::Rate loop_rate(10);
	desiredAngle=angle2Turn;

    while(true)
    {
	if(fabs(currentAngle-desiredAngle)<0.1){break;}
	ros::spinOnce();
	ROS_INFO("Current Angle: %f",currentAngle);
        currentVelocity.angular.z = angularSpd;
	binVelPub_object.publish(currentVelocity);
	
	loop_rate.sleep();
	ROS_INFO("Desired Angle: %f",desiredAngle);
    }
        currentVelocity.angular.z = 0;
}


void exchangeCallback(const team4_ros::readyToExchange::ConstPtr& msg) 
{ 
	float ready=msg->readyToExchange2;
	ros::Rate loop_rate(10);
	if(ready==1){opreadyToEx=true;}
	if (opreadyToEx && readyToEx){
		currentVelocity.linear.x = 1;
        	currentVelocity.angular.z = 1;
		//int counter=0;
		while(ros::ok){
		ros::spinOnce();
		binVelPub_object.publish(currentVelocity);	
		loop_rate.sleep();
		}
		//counter++;}
		//currentVelocity.linear.x = 0;
        	//currentVelocity.angular.z = 0;
		//opreadyToEx=false;
		//readyToEx=false;
		//team4_ros::binIsFull finish_msg; 
		//finish_msg.isFull = true;  
		//finish_object.publish(finish_msg); 
	}	
}

void exchange(){

	rotateAngle(0,1);
	exchange_msg.readyToExchange = 1;  
	exchange_object.publish(exchange_msg);
	readyToEx=true;

}

int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "dog_node_2");
	
	// ROS comms access point 
	//ros::NodeHandle n;
	// master registry pub/sub 
	//bin_pub = n.advertise<team4_ros::binIsFull>("dog_topic",10);
        
	ros::NodeHandle sub_handle; 
	ros::Subscriber mysub_object;
	mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_15/base_pose_ground_truth",1000, groundTruthCallback); 
	
	ros::NodeHandle velPub_handle;
	binVelPub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_15/cmd_vel",1000);
	ros::NodeHandle exn;
	exchange_object = exn.advertise<team4_ros::readyToExchange>("exchange_topic",1000);

	ros::NodeHandle ex_handle;
	ros::Subscriber ex_object = ex_handle.subscribe("exchange_topic",1000,exchangeCallback); 

	//ros::NodeHandle finish_handle; 

	//ros::NodeHandle arrivedHandle;
    //ros::Subscriber arrivedSubscriber=arrivedHandle.subscribe("arrived_topic",1000,arrivedCallBack);

	// master registry pub/sub 
       // finish_object = finish_handle.advertise<team4_ros::binIsFull>("finishRotate_topic",100); 

	// loop 10 Hz 
	ros::Rate loop_rate(10);
	
	while (ros::ok()) 
	{
		ros::spinOnce();
	
		//binVelPub_object.publish(currentVelocity);
		//if(finishRotation){exchange();}
		exchange();
		loop_rate.sleep();

	} 

	return 0; 
}

