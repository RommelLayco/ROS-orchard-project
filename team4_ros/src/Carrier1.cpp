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
#include <team4_ros/binIsFull.h> 
#include <team4_ros/findPicker.h> 
#include <team4_ros/arrived.h>
//#include <team4_ros/readyToUse.h>


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
int id=1;

// Set by sensorCallback when robot is near an obstacle
bool nearCollision;
bool canMove=false;

geometry_msgs::Point desiredLocation;
//counter for overrall collison avoid
int mycounter;
//a sub counter to let picker go back
int sensorCounter;
//currently not used, should be used to detect the object is dynamic or static
bool isDynamic;
//int from 0-60 to detect the object is on left or right
int sensorPoint;

//publisher for arrived
ros::Publisher arrivedPub;
//bool for arrived msg
bool isSent=false;

//bool for publishing arrived msg
bool canPublish=false;

void masterCallback(const team4_ros::findPicker::ConstPtr& msg) 
{ 
	//ROS_INFO("sub echoing pub: %s", msg->data.c_str());
       
        ROS_INFO("Get message from master:");
			if(msg->id==id){
   			desiredLocation.x = msg->x;
   	 		desiredLocation.y = msg->y+2;
    		desiredLocation.z = 0; 
            canPublish=true;
			canMove=true;}
		
      
}

void binCallback(const team4_ros::binIsFull::ConstPtr& msg) 
{ 
	//ROS_INFO("sub echoing pub: %s", msg->data.c_str());
       
        ROS_INFO("sub echoing pub:");

		if(msg->isFull){
   		desiredLocation.x = msg->x;
   	 		desiredLocation.y = msg->y;
    		desiredLocation.z = 0; 
			canMove=true;

		}
      
}

void finishRotateCallback(const team4_ros::binIsFull::ConstPtr& msg){
		if(msg->isFull){
		ROS_INFO("Finish rotating");
   		desiredLocation.x = 10;
   	 		desiredLocation.y = 10;
    		desiredLocation.z = 0; 
			canMove=true;
		}
} 

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    bool isNear = false;
    ROS_INFO("I am carrier1");
    for (i; i < 60; i++) {
         if (msg->ranges[i] <1)
        {
            isNear = true;
            isDynamic=false;
            nearCollision = true;
            ROS_INFO("I'm near something! [%f]", msg->ranges[i]);
            //start collison detection, init two counters to 1.
            ::sensorPoint=i;
            mycounter=1;
            break;
        }

        if (isNear == false)
        {
            nearCollision = false;
        }
        

    }
}


void updateCurrentVelocity() {

    if (nearCollision == true)
    {   
        if(sensorCounter>=0 && sensorCounter<=10){
            currentVelocity.linear.x = 0;
                currentVelocity.angular.z = 0;
                ROS_INFO("I'm in front of a dynamic object,I will wait");
                mypub_object.publish(currentVelocity); 
                sensorCounter++;
                return;
        }
        
            ROS_INFO("It's a static obj,I will move");
            
            if(::sensorPoint>30){
                ROS_INFO("It's on my left,turn right,clock(Z<0) first then anti"); 
                    

                    currentVelocity.linear.x = 0.5;
                    currentVelocity.angular.z=-0.4;
                    
                    mypub_object.publish(currentVelocity); 
                
                
            }else if(::sensorPoint<=30){
             ROS_INFO("It's on my right,turn left,anti(Z>0) first then clock"); 
             
               currentVelocity.linear.x = 0.5;
               currentVelocity.angular.z=0.4;
               mypub_object.publish(currentVelocity); 
           
                
       

   }
       // Let collision resolution take place before we attempt to move towards the goal
   mycounter++;
   return;
}
// keep the speed for collision detection.
 if(mycounter>=1 && mycounter<10){
    mycounter++;
    return;
   }else if(mycounter>=10 && mycounter<=40){
    if(sensorPoint>30){
        //if it first goes to the right,it should goes back to left.
        //number 40,20 can be changed to get better performance
        ROS_INFO("goes back,turn anti Z>0");
        currentVelocity.linear.x = 0.3;
        currentVelocity.angular.z=0.5;
        mypub_object.publish(currentVelocity); 

    }else{
        ROS_INFO("goes back,turn clock Z<0");
         currentVelocity.linear.x = 0.3;
        currentVelocity.angular.z=-0.5;
        mypub_object.publish(currentVelocity); 
    }
    mycounter++;
    return;
   }else if(mycounter>40){
    //stop collison detection,goes back to normal
    mycounter=0;
   }

    // Find the correct 

sensorCounter=0;
    // This is the maximum distance a robot can be from it's
    // desired poisition and still be considered to have reached it
    float distanceThreshold = 2;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;

    ROS_INFO("X distance: [%f]", directionVector.x);
    ROS_INFO("Y distance: [%f]", directionVector.y);

    // Check if we are at the desired location
    if (fabs(directionVector.x) <= distanceThreshold && fabs(directionVector.y) <= distanceThreshold)
    {
        // Robot has reached it's desired location
        // For now, make robot stop. In future, robot should now try to move
        // to the next location on it's path.
        ROS_INFO("I have reached my destination!");
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0.0;

        //if arrived publish the arrvied_msg
        team4_ros::arrived arrived_msg;
       // arrived_msg.isFull=true;
        arrived_msg.x=1;
        if(canPublish){
        ROS_INFO("arrived msg published");
        arrivedPub.publish(arrived_msg);
        canPublish=false;
    }
           



        return;
    }
    
    // Calculate the desired angle
    double desiredAngle = atan2(directionVector.y, directionVector.x);

    // If the desired angle is 0
    if(desiredAngle != 0 && desiredAngle != M_PI)
    {    
        //ROS_INFO("currentAngle is : %f",currentAngle); 
        //ROS_INFO("desiredAngle is : %f",desiredAngle); 

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



int main (int argc, char **argv) 
{
    
  

    nearCollision = false;    

	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "CarrierNode1"); 

	// ROS node hander
	ros::NodeHandle velPub_handle;
	ros::NodeHandle sub_handle; 

	// master registry pub and sub
	//ros::Publisher mypub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
        mypub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_8/cmd_vel",1000);
	ros::Publisher binVelPub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_4/cmd_vel",1000);

	ros::Subscriber mysub_object;
	
	// loop 25 
	ros::Rate loop_rate(10);

	mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_8/base_pose_ground_truth",1000, groundTruthCallback); \
	
	// ROS comms access point 
	ros::NodeHandle n;	
	ros::Publisher carrier_pub;

        ros::Subscriber sub = n.subscribe("robot_8/base_scan", 1000, sensorCallback);

        //team4_ros::readyToUse mypub_msg;

	//ros::Subscriber sub_bin = sub_handle.subscribe("bin_topic",10,binCallback);  

	ros::Subscriber sub_master = sub_handle.subscribe("choosen_carrier",10,masterCallback);

		ros::NodeHandle finish_handle;
        
        // tell master you want to sub to topic 
	ros::Subscriber finish_object = finish_handle.subscribe("finishRotate_topic",100,finishRotateCallback); 
    sensorCounter=0;
    mycounter=0;
    
    //create arrived message
    ros::NodeHandle arrivedHandle;
    arrivedPub=arrivedHandle.advertise<team4_ros::arrived>("arrived_topic",1000);

	
	while (ros::ok()) 
	{ 
		loop_rate.sleep();

		updateCurrentVelocity(); 
      //  mypub_object.publish(currentVelocity); 
        
		// refer to advertise msg type 
                //if(currentVelocity.linear.x == 0 && currentVelocity.angular.z == 0){
			//mypub_msg.isReady = true; 
               // }else{
			//mypub_msg.isReady = false; 
	//	}
		//carrier_pub.publish(mypub_msg);

            if(canMove){
                mypub_object.publish(currentVelocity);
		binVelPub_object.publish(currentVelocity); 
                
            }
		z=0;

		ros::spinOnce();
		loop_rate.sleep();
	} 

	return 0; 
}
