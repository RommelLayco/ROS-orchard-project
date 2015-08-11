#include "ros/ros.h" 
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h" 
#include "geometry_msgs/Twist.h"

ros::Publisher move_pub; 
int x;
float z;

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    ROS_INFO("Sensor:");
    for (i; i < 180; i++) {
         if (msg->ranges[i] < 5)
         {
            ROS_INFO("I'm near something! [%f]", msg->ranges[i]);
            if (i < 45)
            {
                z = 90.0;
            } else if (i >= 45 && i < 90)
            {
                z = 180;
            } else
            {
                z = -90;
            }
            
            
         }
    }
}


void move()
{
    geometry_msgs::Twist move_msg;
    move_msg.linear.x = x; 
    move_msg.linear.y = 0; 
    move_msg.angular.z = z;
    move_pub.publish(move_msg);
}


int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "sensor_node");
	
	// ROS comms access point 
	ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("robot_0/base_scan", 1000, sensorCallback);

    // master registry pub/sub 
	move_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",100);

	// loop 10 Hz 
	ros::Rate loop_rate(10);
	
	x = 1;
	z = 0;
	
    int counter=0;
	while (ros::ok()) 
	{
	    move();
	    z = 0;
		ros::spinOnce();
		loop_rate.sleep();

	} 

	return 0; 
}

