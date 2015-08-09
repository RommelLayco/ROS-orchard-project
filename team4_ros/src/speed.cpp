#include "ros/ros.h"
#include "std_msgs/String.h"



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "changeSpeed");

   
  ros::NodeHandle n;

  //publish to change speed of robot 0
  ros::Publisher speed_pub = n.advertise<std_msgs::String>("/robot_0/cmd_vel", 1000);

  ros::Rate loop_rate(10);

  

  int count = 0;
  while (ros::ok())
  {
    loop_rate.sleep(); 

   
		// move robot
		geometry_msgs::Twist speed_msg;
                speed_msg.linear.x = 1;
                speed_msg.linear.y = 1; 

    		speed_pub.publish(speed_msg);
  }

    return 0;
}
