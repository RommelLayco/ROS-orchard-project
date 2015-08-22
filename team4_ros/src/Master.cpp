
#include <team4_ros/readyToUse.h>
#include <team4_ros/binIsFull.h> 
#include <team4_ros/findPicker.h> 
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

ros::Publisher move_pub; 
int x;
float z;
geometry_msgs::Point desiredLocation;
int readyToUseCarrier[2];
ros::Publisher publishTheChoosenCarrier;


void binCallback(const team4_ros::binIsFull::ConstPtr& msg) 
{ 
	//ROS_INFO("sub echoing pub: %s", msg->data.c_str());
       
        ROS_INFO("sub echoing pub:");

		if(msg->isFull){
   			desiredLocation.x = msg->x;
   	 		desiredLocation.y = msg->y;
    		desiredLocation.z = 0; 

		        team4_ros::findPicker mypub_msg; 
                mypub_msg.x= msg->x;
                mypub_msg.y= msg->y;
				publishTheChoosenCarrier.publish(mypub_msg);

		
		}
      
}

void searchNearestCarrier()
{
	

}


int main (int argc, char **argv) 
{ 

	

	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "master_node");
	
	// ROS comms access point 
	ros::NodeHandle n;


	ros::Subscriber sub_bin = n.subscribe("bin_topic",10,binCallback);  

    // publish a message, identify the id of the carrier need to move
	publishTheChoosenCarrier = n.advertise<team4_ros::findPicker>("choosen_carrier",1000);
	

	// loop 10 Hz 
	ros::Rate loop_rate(10);
	
	x = 1;
	z = 0;

	readyToUseCarrier[0]=0;
	readyToUseCarrier[1]=1;
	
    int counter=0;
	while (ros::ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
		


				

	} 

	return 0; 
}

