#include "ros/ros.h"
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

// Current velocity of the Robot
geometry_msgs::Twist currentVelocity;

// Current location of the robot
geometry_msgs::Pose currentLocation;

// The current angle of the robot
double currentAngle;
double desiredAngle = 0;

// counter
int counter = 0;

// Boolean for the direction of the 
bool VibrateX=false;

// Pub object
ros::Publisher mypub_object;

// Speed and angular velocity when sensor detects something
int x;
float z;

// Set by sensorCallback when robot is near an obstacle
bool nearCollision;

// Index that points to current position in path index
int pathIndex;

geometry_msgs::Point desiredLocations[2];

void generateRandomDesiredLocations(){
    // set up for random number generation
    srand (time(NULL));

    // Setup points on robot's path
    geometry_msgs::Point desiredLocation1;
    //desiredLocation1.x = -10;
    desiredLocation1.x = -8 + rand() % (8 - -8) + 1;
    //desiredLocation1.y = -21;
    desiredLocation1.y = -36 + rand() % (30 - -36) + 1;
    desiredLocation1.z = 0;

    geometry_msgs::Point desiredLocation2;
    //desiredLocation2.x = 10;
    desiredLocation2.x = -8 + rand() % (8 - -8) + 1;
    //desiredLocation2.y = 21;
    desiredLocation2.y = -36 + rand() % (30 - -36) + 1;
    desiredLocation2.z = 0;

    desiredLocations[0] = desiredLocation1;
    desiredLocations[1] = desiredLocation2;


}


void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    bool isNear = false;
    ROS_INFO("Sensor:");
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
    for (i; i < 80; i++) {
        if (msg->ranges[i] < 1.1)
        {
            isNear = true;
            nearCollision = true;
            ROS_INFO("I'm near something! [%f]", msg->ranges[i]);
            

            //Find the angle of the obstacle to the person

            double obstacleAngle = 0;
            if(currentAngle < 3.14){
                obstacleAngle = double(i)/80 * 3.14;
            }
            else{
                obstacleAngle = double(i)/80 * 3.14 + 3.14;
            }


            if (desiredAngle - obstacleAngle > 0.5)
            {
                //move back and spin anticlockwise
                currentVelocity.linear.x = -0.2;
                currentVelocity.angular.z = 2;
                ROS_INFO("i is : %i",i); 

                ROS_INFO("currentAngle is : %f",currentAngle); 
                ROS_INFO("desiredAngle is : %f",desiredAngle); 
                ROS_INFO("obstacleAngle is : %f",obstacleAngle); 
                ROS_INFO("spinning AntiClockwise"); 
            }
            else if(desiredAngle - obstacleAngle < 0.5 && desiredAngle - obstacleAngle >-0.5){

                //move back faster, obstacle at middle.
                 currentVelocity.linear.x = -0.5;
                currentVelocity.angular.z = 0.8;
                 ROS_INFO("i is : %i",i); 

                ROS_INFO("currentAngle is : %f",currentAngle); 
                ROS_INFO("desiredAngle is : %f",desiredAngle); 
                ROS_INFO("obstacleAngle is : %f",obstacleAngle); 
                ROS_INFO("i am stuck! help!");  

            }
            else{
                //move back and spin clockwise
                currentVelocity.linear.x = -0.2;
                currentVelocity.angular.z = -2;
                ROS_INFO("i is : %i",i); 

                ROS_INFO("currentAngle is : %f",currentAngle); 
                ROS_INFO("desiredAngle is : %f",desiredAngle); 
                ROS_INFO("obstacleAngle is : %f",obstacleAngle); 
                ROS_INFO("spinning Clockwise"); 
            }

        }


        mypub_object.publish(currentVelocity);

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
}

void rotateAngle(double desiredAngle)
{
   //Calculate the angle to rotate
    double difference = currentAngle - desiredAngle;
    //Do not rotate if already at desired angle
    if (difference == 0.0){
        return;
    }

     // If the difference between current angle and desired angle is less than 0.5 stop spining
        if (abs(difference) > 0.5)
        {
            ROS_INFO("currentAngle is : %f",currentAngle); 
            ROS_INFO("desiredAngle is : %f",desiredAngle); 
            // Spin
            currentVelocity.linear.x = 0;
            currentVelocity.angular.z = 1;
            
        } else
        {
            // Go forward
            currentVelocity.linear.x = 1;
            currentVelocity.angular.z = 0;
        }
        

}


void updateCurrentVelocity() {


    if (nearCollision == true)
    {
Vibrate();
        // Let collision resolution take place before we attempt to move towards the goal
        return;
    }

    // Find the correct angle

    geometry_msgs::Point desiredLocation = desiredLocations[pathIndex];
    // This is the maximum distance a robot can be from it's
    // desired poisition and still be considered to have reached it
    float distanceThreshold = 0.5;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;

    ROS_INFO("X distance: [%f]", directionVector.x);
    ROS_INFO("Y distance: [%f]", directionVector.y);

    // Check if we are at the desired location
    if (abs(directionVector.x) <= distanceThreshold && abs(directionVector.y) <= distanceThreshold)
    {
        // Robot has reached it's desired location
        // For now, make robot stop. In future, robot should now try to move
        // to the next location on it's path.
        ROS_INFO("I have reached my destination!");
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0.0;
        if (pathIndex < sizeof(desiredLocations) / sizeof(*desiredLocations) - 1)
        {
            pathIndex++;
        }
        else
        {
            // Reset index
            ROS_INFO("Reached final destination, going back to the start");
            
            // call to generate random values for destination
            generateRandomDesiredLocations();

            pathIndex = 0;
        }
        
        return;
    }
    
    // Calculate the desired angle
    desiredAngle = atan2(directionVector.y, directionVector.x) + 3.14;

    rotateAngle(desiredAngle);
 
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
    currentAngle = yaw + 3.14;
    //ROS_INFO("Yaw is : %f",yaw); 
    
}


int main (int argc, char **argv) 
{

    generateRandomDesiredLocations();
  
    pathIndex = 0;


    nearCollision = false;    

    // command line ROS arguments/ name remapping 
    ros::init(argc, argv, "PersonNode"); 

    // ROS node hander
    ros::NodeHandle velPub_handle;
    ros::NodeHandle sub_handle; 
        ros::NodeHandle bark_handle;

    // master registry pub and sub
    //ros::Publisher mypub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
        mypub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_2/cmd_vel",1000);
        ros::Publisher mypub_bark = bark_handle.advertise<std_msgs::String>("person_topic",1000);
    ros::Subscriber mysub_object;
    
    // loop 10 
    ros::Rate loop_rate(10);

    mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_2/base_pose_ground_truth",1000, groundTruthCallback);

        // ROS comms access point 
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("robot_2/base_scan", 1000, sensorCallback);

            
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
