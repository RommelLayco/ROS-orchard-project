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
int vibrateCount = 0; //for vibrate function
int counter = 0;    // for sensor call back function
int aroundCounter = 0; // for goARound function
//counter for overrall collison avoid
int mycounter;
//a sub counter to let picker go back
int sensorCounter;
int updateCounter = 0;
// Boolean for the direction of the 
bool VibrateX=false;

// Pub object
ros::Publisher mypub_object;

// Set by sensorCallback when robot is near an obstacle
bool nearCollision;

// Index that points to current position in path index
int pathIndex;

//int from 0-110 to detect the object is on left or right
int sensorPoint;

geometry_msgs::Point desiredLocations[2];

void generateRandomDesiredLocations(){
    // set up for random number generation
    srand (time(NULL));

    // Setup points on robot's path
    geometry_msgs::Point desiredLocation1;
    //desiredLocation1.x = -10;
    desiredLocation1.x = -5 + rand() % (5 - -5) + 1;
    //desiredLocation1.y = -21;
    desiredLocation1.y = -15 + rand() % (15 - -15) + 1;
    desiredLocation1.z = 0;

    geometry_msgs::Point desiredLocation2;
    //desiredLocation2.x = 10;
    desiredLocation2.x = -5 + rand() % (5 - -5) + 1;
    //desiredLocation2.y = 21;
    desiredLocation2.y = -15 + rand() % (15 - -15) + 1;
    desiredLocation2.z = 0;

    desiredLocations[0] = desiredLocation1;
    desiredLocations[1] = desiredLocation2;

}

/*
angle2Turn is the angle willing to turn in radians, for example, turn 90degrees will be 1.57 radians.
angularSpd should set at 2 to reach max turning speed. positive 2 will be spinning anticlockwise, 
negative 2 will be clockwise.
Eg, turning right 90degrees--- angle2Turn= 1.57, angularSpd = 2
turning left 180degrees --- angle2Turn = 3.14, angularSpd = -2
*/

bool rotateAngle(double angle2Turn, int angularSpd)
{
   //Calculate the angle to rotate
    double destinationAngle = 0;
    currentVelocity.linear.x = 0;
    //int timeLimit = angle2Turn/6.28 * 20;
    ros::Rate loop_rate(10);


    if (angularSpd > 0) //spin anticlockwise
    {
        destinationAngle = currentAngle + angle2Turn;
        /*
        *   Eg: current Angle = 6, turn 1.57 anticlockwise.
        *   6 + 1.57 = 7.57.  7.57 - 6.28 = 1.29
        *   Stop spinning when current angle is closed to 1.29.
        */
        if (destinationAngle > 6.28)
        {
            destinationAngle = destinationAngle - 6.28;
        }
    }
    else //spin clockwise
    {
        //vice versa from above
        destinationAngle = currentAngle - angle2Turn;
        if (destinationAngle < 0)
        {
            destinationAngle = destinationAngle + 6.28;
        }
    }

    while(true)
    {

        if(fabs(currentAngle-destinationAngle)<0.1){break;}

        ros::spinOnce();
        ROS_INFO("Current Angle: %f",currentAngle);
        //ROS_INFO("Desired Angle: %f",desiredAngle);
        currentVelocity.angular.z = angularSpd;
        mypub_object.publish(currentVelocity);
        
        loop_rate.sleep();
        ROS_INFO("destinationAngle: %f",destinationAngle);
    }


    currentVelocity.angular.z = 0;
    return true;

}

void Vibrate() 
{     

    if (vibrateCount > 30)
    {
        vibrateCount = 0;
        return;
    }
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
    vibrateCount++;

}

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    bool isNear = false;
    for (i; i < 110; i++) {
        if (msg->ranges[i] < 1.5)
        {
            isNear = true;
            nearCollision = true;
            ::sensorPoint=i;
            mycounter=1;
            sensorCounter=1;
            Vibrate();
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
        if (vibrateCount < 30)
        {
            ROS_INFO("Vibrate");
            ROS_INFO("counter: [%i]", vibrateCount);
            return;
        }
                // Let collision resolution take place before we attempt to move towards the goal
        if(::sensorPoint>55){
               
            ROS_INFO("It's on my left,turn right,clock(Z<0) first then anti"); 
                    
            if (::sensorPoint < 75)
            {
                ROS_INFO("One Left middle");
                currentVelocity.linear.x = 0;
                currentVelocity.angular.z=-2;
            }
            else
            {
                currentVelocity.linear.x = 0.6;
                currentVelocity.angular.z=-1;
            }

            
                    
            mypub_object.publish(currentVelocity); 
                
                
        }else if(::sensorPoint<=55){

            ROS_INFO("It's on my right,turn left,anti(Z>0) first then clock"); 
             
             if (::sensorPoint > 40)
            {
                ROS_INFO("On right middle");
                currentVelocity.linear.x = 0;
                currentVelocity.angular.z= 2;
            }
            else
            {
                currentVelocity.linear.x = 0.6;
                currentVelocity.angular.z= 1;
            }

            mypub_object.publish(currentVelocity); 
        }
        mycounter++;
        return;
        
    }


    // keep the speed for collision detection.
    if(mycounter>=1 && mycounter<8){

        mycounter++;
        return;

    }
    else if(mycounter>=8 && mycounter<=16)
    {
        if(sensorPoint>55)
        {
            //if it first goes to the right,it should goes back to left.
            //number 40,20 can be changed to get better performance
            ROS_INFO("goes back,turn anti Z>0");
            currentVelocity.linear.x = 0.6;
            currentVelocity.angular.z=1;
            mypub_object.publish(currentVelocity); 

        }
        else
        {
            ROS_INFO("goes back,turn clock Z<0");
            currentVelocity.linear.x = 0.6;
            currentVelocity.angular.z=-1;
            mypub_object.publish(currentVelocity); 
        }
        mycounter++;
        return;
       }
       else if(mycounter>16)
       {
        //stop collison detection,goes back to normal
        mycounter=0;
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

    if(updateCounter > 50){
        generateRandomDesiredLocations();
        ROS_INFO("CHANGE DESTINATION");
        updateCounter=0;
    }
    else{
        ROS_INFO("updateCounter: [%i]", updateCounter);
        updateCounter++;
    }
    if (abs(directionVector.x) <= distanceThreshold && abs(directionVector.y) <= distanceThreshold)
    {
        generateRandomDesiredLocations();
        ROS_INFO("CHANGE DESTINATION");
    }
    /*
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
    */
    // Calculate the desired angle
    desiredAngle = atan2(directionVector.y, directionVector.x) + 3.14;

     //Calculate the angle to rotate
    double difference = currentAngle - desiredAngle;

    bool move = false;

    //Do not rotate if already at desired angle
    if (abs(difference) <0.5){

        currentVelocity.angular.z = 0;
        move = true; //Ok to move
    }
    //rotateAngle(1.57, 2);


    if(!move) //NOT OK to move
    {

        if (difference < 0) // negative rotate anticlockwise
            {
                move = rotateAngle(fabs(difference),2);
            }
            else //positive rotate clockwise
            {
                move = rotateAngle(difference, -2);
            }    
    }
    else{ //OK to move straight.

        currentVelocity.linear.x = 1;
        currentVelocity.angular.z = 0;
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

    sensorCounter=0;
    mycounter=0;
    


            
    while (ros::ok()) 
    { 
        updateCurrentVelocity(); 
        // refer to advertise msg type 
        loop_rate.sleep();
      

        mypub_object.publish(currentVelocity); 

        ros::spinOnce();
        loop_rate.sleep();
    } 

 
    return 0; 
}
