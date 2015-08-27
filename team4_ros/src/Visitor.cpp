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
#include <team4_ros/findVisitor.h>


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
bool canMove = false;

// Set by sensorCallback when robot is near an obstacle
bool nearCollision;

// Index that points to current position in path index
int pathIndex;

geometry_msgs::Point desiredLocations[2];

//counter for overrall collison avoid
int stepCounter = 0;

int mycounter;
//a sub counter to let picker go back
int sensorCounter;

//int from 0-60 to detect the object is on left or right
int sensorPoint;

void guiderCallback(const team4_ros::findVisitor::ConstPtr& msg)
{

    if (msg->isFind)
    {
        canMove = true;
    }

    else
    {
        canMove = false;
    }

}

//Generate a random destination every time this is called.
void generateRandomDesiredLocations()
{
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
    ros::Rate loop_rate(10);


    if (angularSpd > 0) //Spin anticlockwise
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
    else // Spin clockwise
    {
        // Vice versa from above
        destinationAngle = currentAngle - angle2Turn;
        if (destinationAngle < 0)
        {
            destinationAngle = destinationAngle + 6.28;
        }
    }

    while(true)
    {

        if(fabs(currentAngle-destinationAngle)<0.1)
        {
            break;
        }

        ros::spinOnce();
        ROS_INFO("Current Angle: %f",currentAngle);
        currentVelocity.angular.z = angularSpd;
        mypub_object.publish(currentVelocity);

        loop_rate.sleep();
        ROS_INFO("destinationAngle: %f",destinationAngle);
    }


    currentVelocity.angular.z = 0;
    return true;

}


void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    bool isNear = false;
    ROS_INFO("I AM VISITOR");
    for (i; i < 110; i++)
    {
        if (msg->ranges[i] < 1.5)
        {
            isNear = true;
            nearCollision = true;
            ROS_INFO("I'm near something! [%f]", msg->ranges[i]);
            // Start collison detection, init two counters to 1.
            ::sensorPoint = i;
            mycounter = 1;
            break;
        }

        if (isNear == false)
        {
            nearCollision = false;
        }

    }
}


void updateCurrentVelocity()
{

    // Turn and go around obstacle if near something
    if (nearCollision == true)
    {
        if(sensorCounter>=0 && sensorCounter<=10)
        {
            currentVelocity.linear.x = 0;
            currentVelocity.angular.z = 0;
            ROS_INFO("I'm in front of a dynamic object,I will wait");
            mypub_object.publish(currentVelocity);
            sensorCounter++;
            return;
        }

        // Once dancing finished, start going around
        // Let collision resolution take place before we attempt to move towards the goal
        if (::sensorPoint > 55)
        {

            ROS_INFO("It's on my left,turn right,clock(Z<0) first then anti");

            if (::sensorPoint < 75)
            {
                ROS_INFO("One Left middle");
                currentVelocity.linear.x = 0;
                currentVelocity.angular.z =-2;
            }
            else
            {
                currentVelocity.linear.x = 0.6;
                currentVelocity.angular.z = -1;
            }


            mypub_object.publish(currentVelocity);

        }
        else if (::sensorPoint <= 55)
        {

            ROS_INFO("It's on my right,turn left,anti(Z>0) first then clock");

            if (::sensorPoint > 40)
            {
                ROS_INFO("On right middle");
                currentVelocity.linear.x = 0;
                currentVelocity.angular.z = 2;
            }
            else
            {
                currentVelocity.linear.x = 0.6;
                currentVelocity.angular.z = 1;
            }

            mypub_object.publish(currentVelocity);
        }
        mycounter++;
        return;

    }


    // Keep the speed for collision detection.
    if (mycounter >= 1 && mycounter < 8)
    {
        mycounter++;
        return;

    }
    // Moving back to original direction
    else if (mycounter >= 8 && mycounter <= 16)
    {
        if (sensorPoint > 55)
        {
            // If it first goes to the right,it should goes back to left.
            // Number 40,20 can be changed to get better performance
            ROS_INFO("goes back,turn anti Z>0");
            currentVelocity.linear.x = 0.6;
            currentVelocity.angular.z = 1;
            mypub_object.publish(currentVelocity);

        }
        else
        {
            ROS_INFO("goes back,turn clock Z<0");
            currentVelocity.linear.x = 0.6;
            currentVelocity.angular.z = -1;
            mypub_object.publish(currentVelocity);
        }
        mycounter++;
        return;
    }
    else if (mycounter > 16)
    {
        // Stop collison detection, goes back to normal
        mycounter = 0;
    }


    sensorCounter = 0;

    // Find the correct angle

    // This is the maximum distance a robot can be from it's
    // desired poisition and still be considered to have reached it
    geometry_msgs::Point desiredLocation = desiredLocations[pathIndex];

    float distanceThreshold = 0.5;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;

    // Change destination randomly every 50 steps to siulate person wandering
    if (stepCounter > 50)
    {
        generateRandomDesiredLocations();
        ROS_INFO("CHANGE DESTINATION");
        stepCounter = 0;
    }
    else
    {
        ROS_INFO("stepCounter: [%i]", stepCounter);
        stepCounter++;
    }

    // If person arrive destination before 50steps, also change destination
    if (abs(directionVector.x) <= distanceThreshold && abs(directionVector.y) <= distanceThreshold)
    {
        generateRandomDesiredLocations();
        ROS_INFO("CHANGE DESTINATION");
    }
    // Calculate the desired angle
    double desiredAngle = atan2(directionVector.y, directionVector.x);
    // If the desired angle is 0
    if (desiredAngle != 0 && desiredAngle != M_PI)
    {
        // If the deifference between current angle and desired angle is less than 0.1 stop spining
        if (currentAngle - desiredAngle > 0.1 || desiredAngle - currentAngle > 0.1)
        {
            // Spin
            currentVelocity.linear.x = 0;
            currentVelocity.angular.z = 0.5;

        }
        else
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

}



int main (int argc, char **argv)
{

    nearCollision = false;

    // Command line ROS arguments/ name remapping
    ros::init(argc, argv, "Visitor");

    // ROS node hander
    ros::NodeHandle n;

    // Master registry pub and sub
    mypub_object = n.advertise<geometry_msgs::Twist>("robot_19/cmd_vel", 1000);

    // loop 25
    ros::Rate loop_rate(10);

    ros::Subscriber mysub_object = n.subscribe<nav_msgs::Odometry>("robot_19/base_pose_ground_truth",1000, groundTruthCallback);

    // ROS comms access point

    ros::Subscriber sub = n.subscribe("robot_19/base_scan", 1000, sensorCallback);


    ros::Subscriber guiderSub=n.subscribe("findVisitorTopic", 1000, guiderCallback);


    sensorCounter = 0;
    mycounter = 0;

    while (ros::ok())
    {
        loop_rate.sleep();

        updateCurrentVelocity();
        // Refer to advertise msg type
        if (canMove)
        {
            mypub_object.publish(currentVelocity);
        }
        z = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

