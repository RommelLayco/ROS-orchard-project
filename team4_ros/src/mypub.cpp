#include "ros/ros.h" 
#include "sensor_msgs/LaserScan.h" 

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("I heard: [%f]", msg->ranges[0]);
    int size = 180;//sizeof(msg->ranges)/sizeof(msg->ranges[0]);
    int i = 0;
    ROS_INFO("Sensor:");
    for (i; i < size; i++) {
         ROS_INFO("I heard: [%f]", msg->ranges[i]);
    }
}


int main (int argc, char **argv) 
{ 
	// command line ROS arguments/ name remapping 
	ros::init(argc, argv, "name_of_mypub_node"); 

	// ROS comms access point 
	ros::NodeHandle n; 

	// master registry pub/sub 
	//ros::Publisher mypub_object = n.advertise<std_msgs::String>("mypub_topic",100); 

    ros::Subscriber sub = n.subscribe("/robot_0/base_scan", 1000, sensorCallback);

	// loop 10 Hz 
	ros::Rate loop_rate(10); 

    ros::spin();
        
    int counter=0;
	while (ros::ok()) 
	{ 
		loop_rate.sleep(); 
        // refer to advertise msg type 
        //std_msgs::String mypub_msg; 
        //mypub_msg.data = "hello world"; 
        //mypub_object.publish(mypub_msg); 

	} 

	return 0; 
}

