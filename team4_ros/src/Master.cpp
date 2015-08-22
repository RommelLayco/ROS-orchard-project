#include "ros/ros.h" 
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h" 
#include "geometry_msgs/Twist.h"
#include <team4_ros/readyToUse.h>
#include <team4_ros/binIsFull.h> 
#include <list> 

ros::Publisher move_pub; 
int x;
float z;
geometry_msgs::Point desiredLocation;
typedef list<int> LISTINT; 
LISTINT listOne;
LISTINT::iterator i; 


void binCallback(const team4_ros::binIsFull::ConstPtr& msg) 
{ 
	//ROS_INFO("sub echoing pub: %s", msg->data.c_str());
       
        ROS_INFO("sub echoing pub:");

		if(msg->isFull){
   			desiredLocation.x = msg->x;
   	 		desiredLocation.y = msg->y;
    		desiredLocation.z = 0; 

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

    ros::Subscriber sub = n.subscribe("bin_topic", 1000, binCallback);

    // publish a message, identify the id of the carrier need to move

	

	// loop 10 Hz 
	ros::Rate loop_rate(10);
	
	x = 1;
	z = 0;
	
    int counter=0;
	while (ros::ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();

	} 

	return 0; 
}

