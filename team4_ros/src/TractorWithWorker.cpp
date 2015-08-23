//counter variable in sensorCallback#include "ros/ros.h"
#include <stdlib.h>
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <queue>
#include <sstream>
#include "math.h"
#include <unistd.h>
#include <time.h> 
#include <fstream>
	

using namespace std;
// Current velocity of the Robot
geometry_msgs::Twist currentVelocity;

// Current location of the robot
geometry_msgs::Pose currentLocation;

// The current angle of the robot
double currentAngle;
double desiredAngle;


// counter
int timeCount = 0;	//for rotatAngle function
int counter = 0;	//for sensorCallback function


// Pub object
ros::Publisher mypub_object;

// Speed and angular velocity when sensor detects something
int x;
float z;
//counter for desiredAngles
int i;
// Set by sensorCallback when robot is near an obstacle
bool nearCollision;

// Index that points to current position in path index
int pathIndex;

geometry_msgs::Point desiredLocations[4];

float desiredAngles[4] = {0.00, -1.57, -3.14, -4.71 };
void generateDesiredLocations(){
    // set up for random number generation
    srand (time(NULL));
	 
	const int ROWS = 4;
	const int COLS = 2;
	const int BUFFSIZE = 80;
	 
	//Read position values from tractorLocations
	float positions[ROWS][COLS];
	char buff[BUFFSIZE]; // a buffer to temporarily park the data
	ifstream infile("../../locations/tractorLocations");
	stringstream ss;
	for( int row = 0; row < ROWS; ++row ) {
		// read a full line of input into the buffer (newline is
		//  automatically discarded)
		infile.getline( buff,  BUFFSIZE );
	    // copy the entire line into the stringstream
	    ss << buff;
	    for( int col = 0; col < COLS; ++col ) {	     
      	  ss.getline( buff, 6, ' ' );	    
	      positions[row][col] = atof( buff );
	    }
	    
	    ss << "";

	    ss.clear();
  	}

    // Setup points on robot's path
    geometry_msgs::Point desiredLocation1;
    desiredLocation1.x = positions[0][0];
    desiredLocation1.y = positions[0][1];
    desiredLocation1.z = 0;

    geometry_msgs::Point desiredLocation2;
    desiredLocation2.x = positions[1][0];
    desiredLocation2.y = positions[1][1];;
    desiredLocation2.z = 0;

	geometry_msgs::Point desiredLocation3;
    desiredLocation3.x = positions[2][0];;
    desiredLocation3.y = positions[2][1];;
    desiredLocation3.z = 0;

	geometry_msgs::Point desiredLocation4;
    desiredLocation4.x = positions[3][0];
    desiredLocation4.y = positions[3][1];
    desiredLocation4.z = 0;

    desiredLocations[0] = desiredLocation1;
    desiredLocations[1] = desiredLocation2;
	desiredLocations[2] = desiredLocation3;
	desiredLocations[3] = desiredLocation4;

}

/*
void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    bool isNear = false;
    //counter to 15 to give time to move back
    if(nearCollision){
        counter++;

        //after 15, reinvoke the sensor function
        if (counter > 15){
            counter = 0;
            nearCollision = false;
        }
        return;
    }
    for (i; i < 110; i++) {
        if (msg->ranges[i] < 1.1)
        {
            isNear = true;
            nearCollision = true;
          
            ROS_INFO("This thing is cool!"); 


            //Find the angle of the obstacle to the person

            double obstacleAngle = 0;
            if(currentAngle < 3.14){
                obstacleAngle = double(i)/110 * 3.14;
            }
            else{
                obstacleAngle = double(i)/110 * 3.14 + 3.14;
            }


            if (desiredAngle - obstacleAngle > 0.5)
            {
                //move back and spin anticlockwise
                currentVelocity.linear.x = -0.2;
                currentVelocity.angular.z = 2;


            }
            else if(desiredAngle - obstacleAngle < 0.5 && desiredAngle - obstacleAngle >-0.5){

                //move back faster, obstacle at middle.
                 currentVelocity.linear.x = -0.5;
                currentVelocity.angular.z = 0.8;


            }
            else{
                //move back and spin clockwise
                currentVelocity.linear.x = -0.2;
                currentVelocity.angular.z = -2;

            }

        }


        mypub_object.publish(currentVelocity);

    }
}

*/


void rotateAngle(double desiredAngle, double angularSpd)
{
	//Calculate the angle to rotate
	currentVelocity.linear.x = 0;
	//int timeLimit = angle2Turn/6.28 * 20;
	ros::Rate loop_rate(100);	
	while(true)
	{
		//int d=currentAngle-desiredAngle;
		if(fabs(currentAngle-desiredAngle)<0.01){break;}
		ros::spinOnce();
		ROS_INFO("Current Angle: %f",currentAngle);
	
		currentVelocity.angular.z = angularSpd;
		mypub_object.publish(currentVelocity);
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

void updateCurrentVelocity() {
ROS_INFO("currentAngle is : %f",currentAngle); 
	/*
    if (nearCollision == true)
    {    
        // Let collision resolution take place before we attempt to move towards the goal
        return;
    }*/

    // Find the correct angle

    geometry_msgs::Point desiredLocation = desiredLocations[pathIndex];
    // This is the maximum distance a robot can be from it's
    // desired poisition and still be considered to have reached it
    float distanceThreshold = 0.5;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;

   // ROS_INFO("X distance: [%f]", directionVector.x);
   // ROS_INFO("Y distance: [%f]", directionVector.y);

    // Check if we are at the desired location
    if (abs(directionVector.x) <= distanceThreshold && abs(directionVector.y) <= distanceThreshold)
    {
        // Robot has reached it's desired location
        // For now, make robot stop. In future, robot should now try to move
        // to the next location on it's path.
        ROS_INFO("I have reached my destination!");
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0.0;
        if (pathIndex < (sizeof(desiredLocations) / sizeof(*desiredLocations)) )
        {
            pathIndex++;
			ROS_INFO("Path Index: %d",pathIndex);
	    	//rotate 90 degrees right	
			
	    	rotateAngle(desiredAngles[i],-0.1);
				
			i++;
        }
        else
        {
            // Reset index
            ROS_INFO("Reached final destination, going back to the start");                       
            pathIndex = 0;
			i =0;
        }
        
        return;
    }else{currentVelocity.linear.x =1;}
	
	
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

    generateDesiredLocations();
  
    pathIndex = 0;
    

    // command line ROS arguments/ name remapping 
    ros::init(argc, argv, "TractorWithWorkerNode"); 

    // ROS node hander
    ros::NodeHandle velPub_handle;
    ros::NodeHandle sub_handle; 
        ros::NodeHandle bark_handle;

    // master registry pub and sub
    //ros::Publisher mypub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
        mypub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_6/cmd_vel",1000);
        //ros::Publisher mypub_bark = bark_handle.advertise<std_msgs::String>("person_topic",1000);
    ros::Subscriber mysub_object;
    
    // loop 10 
    ros::Rate loop_rate(10);

    mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_6/base_pose_ground_truth",1000, groundTruthCallback);

        // ROS comms access point 
    ros::NodeHandle n;

    //ros::Subscriber sub = n.subscribe("robot_4/base_scan", 1000, sensorCallback);

            
    while (ros::ok()) 
    { 

        updateCurrentVelocity(); 
        // refer to advertise msg type 
        loop_rate.sleep();

        mypub_object.publish(currentVelocity); 
        z=0;

        ros::spinOnce();
        loop_rate.sleep();
    } 

 
    return 0; 
}
