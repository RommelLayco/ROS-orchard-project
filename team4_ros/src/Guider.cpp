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

// Set by sensorCallback when robot is near an obstacle
bool nearCollision;
bool isFindVisitor=false;

bool VibrateX=false;
bool needVibrate=false;
int counter=0;

//counter for overrall collison avoid
int mycounter;
//a sub counter to let picker go back
int sensorCounter;

//int from 0-60 to detect the object is on left or right
int sensorPoint;
// Index that points to current position in path index

ros::Publisher pubToVisitor;

geometry_msgs::Point desiredLocation;




/*
angle2Turn is the angle willing to turn in radians, for example, turn 90degrees will be 1.57 radians.
angularSpd should set at 2 to reach max turning speed. positive 2 will be spinning anticlockwise,
negative 2 will be clockwise.
Eg, turning right 90degrees--- angle2Turn= 1.57, angularSpd = 2
turning left 180degrees --- angle2Turn = 3.14, angularSpd = -2
*/

bool rotateAngle(double angle2Turn, int angularSpd)
{
    // Calculate the angle to rotate
    double destinationAngle = 0;
    currentVelocity.linear.x = 0;
    ros::Rate loop_rate(10);


    if (angularSpd > 0) // Spin anticlockwise
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

        if(fabs(currentAngle-destinationAngle) < 0.1)
        {
            break;
        }

        ros::spinOnce();
        ROS_INFO("Current Angle: %f", currentAngle);
        currentVelocity.angular.z = angularSpd;
        mypub_object.publish(currentVelocity);

        loop_rate.sleep();
        ROS_INFO("destinationAngle: %f", destinationAngle);
    }


    currentVelocity.angular.z = 0;
    return true;

}


void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int i = 0;
    bool isNear = false;
    ROS_INFO("I AM GUIDER:");
    for (i; i < 110; i++)
    {
        if (msg->ranges[i] < 1.5)
        {
            isNear = true;
            nearCollision = true;
            ROS_INFO("I'm near something! [%f]", msg->ranges[i]);
            // Start collison detection, init two counters to 1.
            ::sensorPoint=i;
            mycounter = 1;
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
    // Update Current Position

    if (VibrateX == false)
    {
        currentVelocity.angular.z = 0.95;
        VibrateX = true;
    }
    else
    {
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = -1.0;
        VibrateX = false;
    }

    if (counter > 5)
    {
        counter = 0;
        needVibrate = false;
        return;
    }
    counter = counter + 1;
}

void publishMessageToVisitor()
{
    team4_ros::findVisitor mypub_msg;
    mypub_msg.isFind = isFindVisitor;
    pubToVisitor.publish(mypub_msg);
}


void updateCurrentVelocity()
{

    // Turn and go around obstacle if near something
    if (nearCollision == true)
    {
        if(sensorCounter >= 0 && sensorCounter <= 10)
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
        if(::sensorPoint > 55)
        {
            ROS_INFO("It's on my left,turn right,clock(Z<0) first then anti");

            if (::sensorPoint < 75)
            {
                ROS_INFO("One Left middle");
                currentVelocity.linear.x = 0;
                currentVelocity.angular.z = -2;
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
        // Stop collison detection,goes back to normal
        mycounter = 0;
    }


    sensorCounter = 0;

    // Find the correct angle

    // This is the maximum distance a robot can be from it's
    // desired poisition and still be considered to have reached it
    float distanceThreshold = 2;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - currentLocation.position.x;
    directionVector.y = desiredLocation.y - currentLocation.position.y;
    directionVector.z = desiredLocation.z - currentLocation.position.z;


    // Check if we are at the desired location
    if (fabs(directionVector.x) <= distanceThreshold && fabs(directionVector.y) <= distanceThreshold)
    {
        // Robot has reached it's desired location
        // For now, make robot stop. In future, robot should now try to move
        // to the next location on it's path.
        currentVelocity.linear.x = 0;
        currentVelocity.angular.z = 0.0;
        ROS_INFO("Reached destination");
        needVibrate = true;
        isFindVisitor = true;

        return;
    }

    isFindVisitor = false;
    // Calculate the desired angle
    double desiredAngle = atan2(directionVector.y, directionVector.x);
    // If the desired angle is 0
    if (desiredAngle != 0 && desiredAngle != M_PI)
    {

        // If the difference between current angle and desired angle is less than 0.1 stop spining
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
    // Update Current Position
    currentLocation = msg.pose.pose;
    double x = currentLocation.orientation.x;
    double y = currentLocation.orientation.y;
    double z = currentLocation.orientation.z;
    double w = currentLocation.orientation.w;

    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
    currentAngle = yaw;

}

void visitorGroundTruthCallback(const nav_msgs::Odometry msg)
{
    // Update Current Position
    desiredLocation.x = msg.pose.pose.position.x;
    desiredLocation.y = msg.pose.pose.position.y;

}



int main (int argc, char **argv)
{

    nearCollision = false;

    // Command line ROS arguments/ name remapping
    ros::init(argc, argv, "Guider");

    // ROS node hander
    ros::NodeHandle n;

    // Master registry pub and sub
    mypub_object = n.advertise<geometry_msgs::Twist>("robot_18/cmd_vel",1000);

    pubToVisitor =n.advertise<team4_ros::findVisitor>("findVisitorTopic",1000);

    // loop 25
    ros::Rate loop_rate(10);

    ros::Subscriber mysub_object = n.subscribe<nav_msgs::Odometry>("robot_18/base_pose_ground_truth",1000, groundTruthCallback);
    ros::Subscriber visitorSub = n.subscribe<nav_msgs::Odometry>("robot_19/base_pose_ground_truth",1000, visitorGroundTruthCallback);

    // ROS comms access point

    ros::Subscriber sub = n.subscribe("robot_18/base_scan", 1000, sensorCallback);
    sensorCounter=0;
    mycounter=0;

    while (ros::ok())
    {
        loop_rate.sleep();

        if (needVibrate)
        {
            Vibrate();
        }
        else
        {
            updateCurrentVelocity();
        }

        publishMessageToVisitor();
        // Refer to advertise msg type

        mypub_object.publish(currentVelocity);
        z = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

