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



// Current velocity of the Robot
geometry_msgs::Twist currentVelocity;

// Current location of the robot
geometry_msgs::Pose currentLocation;

// The current angle of the robot
double currentAngle;

// Pub object
ros::Publisher mypub_object;
ros::Publisher binVelPub_object;

// Speed and angular velocity when sensor detects something
int x;
float z;

// Set by sensorCallback when robot is near an obstacle
bool nearCollision;

// Index that points to current position in path index
int pathIndex;

// Boolean for the direction of the vibrate
bool VibrateX=false;

// When the bin is full, stop the robot and the bin
bool isFull=false;
geometry_msgs::Point desiredLocations[2];

//counter for overrall collison avoid
int mycounter;
//a sub counter to let picker go back
int sensorCounter;
//currently not used, should be used to detect the object is dynamic or static
bool isDynamic;
//int from 0-60 to detect the object is on left or right
int sensorPoint;

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    bool isNear = false;
    for (i; i < 60; i++) {
        if (msg->ranges[i] <1.5)
        {
            isNear = true;
            isDynamic=false;
            nearCollision = true;
            ROS_INFO("I'm near something! [%f]", msg->ranges[i]);
            //start collison detection, init two counters to 1.
            ::sensorPoint=i;
            mycounter=1;
            sensorCounter=1;
            break;
        }
        if (isNear == false)
        {
            nearCollision = false;
             
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
}

void binCallback(const team4_ros::binIsFull::ConstPtr& msg) 
{ 
    //ROS_INFO("sub echoing pub: %s", msg->data.c_str());

        //ROS_INFO("sub echoing pub: %s",msg->isFull);

    if(msg->isFull){
       isFull=true;
       ROS_INFO("AAAAAAAAAA");
   }

}


void updateCurrentVelocity() {

    if (nearCollision == true)
    {   //this part is not used.
        if(isDynamic){
            int c=0;
            while(c<50){
                currentVelocity.linear.x = 0;
                currentVelocity.angular.z = 0;
                ROS_INFO("I'm in front of a dynamic object,I will wait");
                c++;
            }
            isDynamic=false;
        }
        else{
            ROS_INFO("It's a static obj,I will move");
            
            if(::sensorPoint>30){
                ROS_INFO("It's on my left,turn right,clock(Z<0) first then anti"); 
                    

                    currentVelocity.linear.x = 0.5;
                    currentVelocity.angular.z=-0.5;
                    
                    mypub_object.publish(currentVelocity); 
                
                
            }else if(::sensorPoint<=30){
             ROS_INFO("It's on my right,turn left,anti(Z>0) first then clock"); 
             
               currentVelocity.linear.x = 0.5;
               currentVelocity.angular.z=0.5;
               mypub_object.publish(currentVelocity); 
           
                
       }

   }
    // Let collision resolution take place before we attempt to move towards the goal
   mycounter++;
   return;
}
// keep the speed for collision detection.
 if(mycounter>=1 && mycounter<20){
    mycounter++;
    return;
   }else if(mycounter>=20 && mycounter<=40){
    if(sensorPoint>30){
        //if it first goes to the right,it should goes back to left.
        //number 40,20 can be changed to get better performance
        ROS_INFO("goes back,turn anti Z>0");
        currentVelocity.linear.x = 0.5;
        currentVelocity.angular.z=0.5;
        mypub_object.publish(currentVelocity); 

    }else{
        ROS_INFO("goes back,turn clock Z<0");
         currentVelocity.linear.x = 0.5;
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


geometry_msgs::Point desiredLocation = desiredLocations[pathIndex];
    // This is the maximum distance a robot can be from it's
    // desired poisition and still be considered to have reached it
float distanceThreshold = 0.5;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;

    //ROS_INFO("X distance: [%f]", currentLocation.position.x);
    //ROS_INFO("Y distance: [%f]", currentLocation.position.y);

    // Check if we are at the desired location
    if (fabs(directionVector.x) <= distanceThreshold && fabs(directionVector.y) <= distanceThreshold)
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
            sleep(2);
            ROS_INFO("Reached destination");
        }
        else
        {
            // Reset index
            ROS_INFO("Reached final destination");

            Vibrate();
            pathIndex=0;
            
        }
        
        return;
    }
    
    // Calculate the desired angle
    double desiredAngle = atan2(directionVector.y, directionVector.x);
    
    if (z != 0)
    {
        currentVelocity.linear.x = 1;
        currentVelocity.angular.z = z;
        return;
    }

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
            currentVelocity.linear.x = 1.5;

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

    // Setup points on robot's path
    geometry_msgs::Point desiredLocation1;
    //desiredLocation1.x = -10;
    desiredLocation1.x = -1.75;
    //desiredLocation1.y = -21;
    desiredLocation1.y = 25;
    desiredLocation1.z = 0;

    geometry_msgs::Point desiredLocation2;
    //desiredLocation2.x = 10;
    desiredLocation2.x = -1.75;
    //desiredLocation2.y = 21;
    desiredLocation2.y = 32;
    desiredLocation2.z = 0;

    desiredLocations[0] = desiredLocation1;
    desiredLocations[1] = desiredLocation2;

    pathIndex = 0;


    nearCollision = false;  

    

    // command line ROS arguments/ name remapping 
    ros::init(argc, argv, "PickerNode"); 

    // Control robot 0
    ros::NodeHandle velPub_handle;
    mypub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);

    // control robot 1
    ros::Publisher binVelPub_object = velPub_handle.advertise<geometry_msgs::Twist>("robot_3/cmd_vel",1000);
    
    // Subscribe the ground truth of robot 1
    ros::NodeHandle sub_handle; 
    ros::Subscriber mysub_object;
    mysub_object = sub_handle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000, groundTruthCallback);
    
    // Sensor 
    ros::NodeHandle n;
    ros::Subscriber sensorSub = n.subscribe("robot_0/base_scan", 1000, sensorCallback);

    //ros::Subscriber sub = n.subscribe("robot_0/base_scan", 1000, sensorCallback);
    ros::Subscriber sub_bin = sub_handle.subscribe("bin_topic",10,binCallback); 
    // loop 25 
    ros::Rate loop_rate(10);

    sensorCounter=0;
    mycounter=0;
    



    while (ros::ok()) 
    { 
        loop_rate.sleep();
        
        updateCurrentVelocity(); 
        mypub_object.publish(currentVelocity); 
        binVelPub_object.publish(currentVelocity);

        if(nearCollision){
            

          //
        }else{
       // updateCurrentVelocity(); 
            if(!isFull){
                mypub_object.publish(currentVelocity); 
                binVelPub_object.publish(currentVelocity);
            }
        }

        // refer to advertise msg type 

        z=0;

        ros::spinOnce();
    } 

    return 0; 
}
