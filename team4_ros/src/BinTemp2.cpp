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
ros::Publisher bin_pub;
ros::Publisher binVelPub_object;
ros::Publisher exchange_object;
ros::Publisher finish_object;
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
	//geometry_msgs::Pose currentLocation;
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

void rotateAngle(double angle2Turn, int angularSpd)
{
   //Calculate the angle to rotate
   
    currentVelocity.linear.x = 0;
    //int timeLimit = angle2Turn/6.28 * 20;
	ros::Rate loop_rate(10);
	desiredAngle=angle2Turn;
        //currentAngle=1.57;

    while(true)
    {
	//int d=currentAngle-desiredAngle;
	if(fabs(currentAngle-desiredAngle)<0.05){break;}
	ros::spinOnce();
	ROS_INFO("Current Angle: %f",currentAngle);
	//ROS_INFO("Desired Angle: %f",desiredAngle);
        currentVelocity.angular.z = angularSpd;
	binVelPub_object.publish(currentVelocity);
	
	loop_rate.sleep();
	ROS_INFO("Desired Angle: %f",desiredAngle);
        //timeCount++;
        //return false;
    }
        ROS_INFO("Time Count will be RESET NOW");

        currentVelocity.angular.z = 0;
        //timeCount = 0;
        //return true;

}


void exchangeCallback(const team4_ros::readyToExchange::ConstPtr& msg) 
{ 
	ROS_INFO("sub echoing pub");
	float ready=msg->readyToExchange2;
	ros::Rate loop_rate(10);
	if(ready==1){opreadyToEx=true;}
	if (opreadyToEx && readyToEx){
		//readyToEx=true;
		ROS_INFO("Yes");
		currentVelocity.linear.x = 1;
        	currentVelocity.angular.z = 1;
		int counter=0;
		while(counter<160){
		ros::spinOnce();
		binVelPub_object.publish(currentVelocity);	
		loop_rate.sleep();
		counter++;}
		currentVelocity.linear.x = 0;
        	currentVelocity.angular.z = 0;
		opreadyToEx=false;
		readyToEx=false;
		//team4_ros::finishRotate finish_msg; 
		//finish_msg.isFull = true;  
		//finish_object.publish(finish_msg); 
		team4_ros::binIsFull finish_msg; 
		finish_msg.isFull = true;  
		finish_object.publish(finish_msg); 
	}
	//else{readyToEx=false;}	
}

void exchange()
{
	//if (currentAngle==3.14 || currentAngle==0){
	rotateAngle(0.1,1);

	exchange_msg.readyToExchange = 1;  
	exchange_object.publish(exchange_msg);
	ROS_INFO("sub ec");
	readyToEx=true;

}
void arrivedCallBack(const team4_ros::arrived::ConstPtr& msg){
	if(msg->x==1){
		exchange();
	}
}

int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "bin_node_t2");
	
	// ROS comms access point 
	ros::NodeHandle n;
	// master registry pub/sub 
	bin_pub = n.advertise<team4_ros::binIsFull>("bin_topic",10);
        
	ros::NodeHandle sub_handle; 
	ros::Subscriber mysub_object;
	mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_4/base_pose_ground_truth",1000, groundTruthCallback); 
	
	ros::NodeHandle velPub_handle;
	binVelPub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_4/cmd_vel",1000);
	ros::NodeHandle exn;
	exchange_object = exn.advertise<team4_ros::readyToExchange>("exchange_topic",1000);

	ros::NodeHandle ex_handle;
	ros::Subscriber ex_object = ex_handle.subscribe("exchange_topic",1000,exchangeCallback); 

	ros::NodeHandle finish_handle; 

	ros::NodeHandle arrivedHandle;
    ros::Subscriber arrivedSubscriber=arrivedHandle.subscribe("arrived_topic",1000,arrivedCallBack);

	// master registry pub/sub 
        finish_object = finish_handle.advertise<team4_ros::binIsFull>("finishRotate_topic",100); 

	// loop 10 Hz 
	ros::Rate loop_rate(10);
	
	while (ros::ok()) 
	{
		ros::spinOnce();
	
		//binVelPub_object.publish(currentVelocity);
		//if(finishRotation){exchange();}
		loop_rate.sleep();

	} 

	return 0; 
}

