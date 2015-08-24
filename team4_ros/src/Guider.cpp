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
#include <team4_ros/findVisitor.h> 


// Current velocity of the Robot
geometry_msgs::Twist currentVelocity;

// Current location of the robot
geometry_msgs::Pose currentLocation;

// The current angle of the robot
double currentAngle;

// Pub object
ros::Publisher mypub_object;

// Speed and angular velocity when sensor detects something
int x;
float z;

// Set by sensorCallback when robot is near an obstacle
bool nearCollision;
bool isFindVisitor=false;

bool VibrateX=false;
bool needVibrate=false;
int counter=0;

// Index that points to current position in path index

ros::Publisher pubToVisitor;

geometry_msgs::Point desiredLocation;

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    bool isNear = false;
    ROS_INFO("i AM GUIDER:");
    for (i; i < 60; i++) {
        if (msg->ranges[i] < 1)
        {
            isNear = true;
            nearCollision = true;
            ROS_INFO("I'm near something! [%f]", msg->ranges[i]);

            if (i < 20)
            {
                // Spin to the left
                ROS_INFO("Spinning left");
                currentVelocity.linear.x = 0.5;
                currentVelocity.angular.z = 1;
            } else if (i >= 20 && i < 40)
            {
                // Move backwards and spin right
                ROS_INFO("Moving backwards and spinning right");
                currentVelocity.linear.x = -0.5;
                currentVelocity.angular.z = -0.5;
            } else
            {
                // Spin to the right
                ROS_INFO("Spinning right");
                currentVelocity.linear.x = 0.5;
                currentVelocity.angular.z = -1;
            }

        }

        if (isNear == false)
        {
            nearCollision = false;
            //currentVelocity.linear.x = 0;
            //currentVelocity.angular.z = 0.0;
        }
        

    }
}




void Vibrate() 
{     
    //Update Current Position
    
    if (VibrateX == false){
	//currentVelocity.linear.y = 3;
    currentVelocity.angular.z = 0.95;
	VibrateX = true;
	}
	else {
	currentVelocity.linear.x = 0;
        currentVelocity.angular.z = -1.0;

        VibrateX = false;
	}
	if(counter>5){
	counter=0;
	needVibrate=false;
	return;	
	
	}
	counter=counter+1;
}

void publishMessageToVisitor(){
	
		team4_ros::findVisitor mypub_msg; 
		mypub_msg.isFind=isFindVisitor;
		pubToVisitor.publish(mypub_msg);
		
		
}


void updateCurrentVelocity() {

    if (nearCollision == true)
    {
        // Let collision resolution take place before we attempt to move towards the goal
        return;
    }

    // Find the correct angle

    // This is the maximum distance a robot can be from it's
    // desired poisition and still be considered to have reached it
    float distanceThreshold = 2;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;


    // Check if we are at the desired location
    if (fabs(directionVector.x) <= distanceThreshold && fabs(directionVector.y) <= distanceThreshold)
    {
        // Robot has reached it's desired location
        // For now, make robot stop. In future, robot should now try to move
        // to the next location on it's path.
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0.0;
		ROS_INFO("Reached destination");
		needVibrate=true;
		isFindVisitor=true;
		

        
        return;
    }

	
    isFindVisitor=false;
    // Calculate the desired angle
    double desiredAngle = atan2(directionVector.y, directionVector.x);
    // If the desired angle is 0
    if(desiredAngle != 0 && desiredAngle != M_PI)
    {    

        // If the deifference between current angle and desired angle is less than 0.1 stop spining
        if (currentAngle - desiredAngle > 0.1 || desiredAngle - currentAngle > 0.1)
        {
            // Spin
            currentVelocity.linear.x = 0;
            currentVelocity.angular.z = 0.5;
            
        } else
        {
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

void visitorGroundTruthCallback(const nav_msgs::Odometry msg) 
{     
    //Update Current Position
    desiredLocation.x = msg.pose.pose.position.x; 
	desiredLocation.y = msg.pose.pose.position.y; 
	
}



int main (int argc, char **argv) 
{

    nearCollision = false;    

	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "Guider"); 

	// ROS node hander
	ros::NodeHandle n; 

	// master registry pub and sub
    mypub_object = n.advertise<geometry_msgs::Twist>("robot_12/cmd_vel",1000);

	pubToVisitor =n.advertise<team4_ros::findVisitor>("findVisitorTopic",1000);
	
	// loop 25 
	ros::Rate loop_rate(10);

	ros::Subscriber mysub_object = n.subscribe<nav_msgs::Odometry>("robot_12/base_pose_ground_truth",1000, groundTruthCallback); 
	ros::Subscriber visitorSub = n.subscribe<nav_msgs::Odometry>("robot_11/base_pose_ground_truth",1000, visitorGroundTruthCallback); 
	
	// ROS comms access point 

    ros::Subscriber sub = n.subscribe("robot_12/base_scan", 1000, sensorCallback);


	while (ros::ok()) 
	{ 
		loop_rate.sleep();


		if(needVibrate){
		Vibrate();
		}
		else {
		updateCurrentVelocity(); 
		}
	
		publishMessageToVisitor();
		// refer to advertise msg type 

			
		mypub_object.publish(currentVelocity); 
		z=0;

		ros::spinOnce();
		loop_rate.sleep();
	} 

	return 0; 
}
