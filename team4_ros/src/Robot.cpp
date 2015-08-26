#ifndef ROBOT_CPP
#define ROBOT_CPP

#include "Robot.h"
#include "Util.cpp"
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

using namespace std;

// Contructor
Robot::Robot(int sensor_range, int sensor_angle, int number, std::string type)
{
    // Initialise state to default
    current_x = 0;
    current_y = 0;
    current_theta = 0;
    linear_velocity_x = 0;
    linear_velocity_y = 0;
    angular_velocity = 0;
    current_state = Orienting;
    goalIndex = 0;

    sensorRange = sensor_range;
    sensorAngle = sensor_angle;

    // Get id of node
    int id = Util::getNextId();
    std::string baseString = "robot_" + std::to_string(id);

    // set the id of the robot for writing to file purposes
    unique_id = number;
    robotType = type;

    // Setup publisher and subscribers
    positionPub = publisherHandle.advertise<geometry_msgs::Twist>(baseString + "/cmd_vel", 1000);
    groundtruthSub = subscriberHandle.subscribe<nav_msgs::Odometry>(baseString + "/base_pose_ground_truth", 1000, &Robot::positionCallback, this);
    sensorSub = subscriberHandle.subscribe(baseString + "/base_scan", 1000, &Robot::sensorCallback, this);

    ROS_INFO("Robot [%d] instantiated", id);


    mycounter=0;
    direction=Left;

}

double Robot::getXPos()
{
    return current_x;
}

double Robot::getYPos()
{
    return current_y;
}


void Robot::addGoal(geometry_msgs::Point goal)
{
    ROS_INFO("X: %f", goal.x);
    ROS_INFO("Y: %f", goal.y);
    goals.push_back(goal);
    ROS_INFO("size: %lu", goals.size());


}

void Robot::addSpeedListener(SpeedListener* listener)
{
    speedListeners.push_back(listener);
}

void Robot::addPositionListener(PositionListener* listener)
{
    positionListeners.push_back(listener);
}

robotState Robot::getState()
{
    return this->current_state;
}

void Robot::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorMsg)
{// Handle sensor data
    int left_vals = sensorAngle / 2;
    int right_vals = sensorAngle - left_vals;

    int i = 0;
    bool isNear = false;
    // Loop through sensor data array
    for (i; i < sensorAngle; i++) {
        // If an object is detected within sensorRange,
        // determine it's position relative to the entity
        if (sensorMsg->ranges[i] < sensorRange)
        {
            isNear = true;
            mycounter=1;
            current_state = CollisionResolution; // Entity is now in CollisionResolution state
            CollisionType type = getCollisionType(i, sensorRange, sensorMsg->ranges[i]);
            if (i < left_vals)
            {
                // Collision is on right
                direction=Right;
                rightCollisionDetected(type);
                break;
            } else if (i >= left_vals && i < sensorAngle)
            {
                // Collision is in front
                direction=Left;
                centerCollisionDetected(type);
                break;
            } else
            {
                // Collision is on left
                direction=Left;
                leftCollisionDetected(type);
                break;
            }

        }

        if (isNear == false)
        {
            current_state = Orienting; // No collision was detected, current state is Orienting
        }
        
        notifySpeedListeners();

    }
}


CollisionType Robot::getCollisionType(int sensorIndex, int sensorRange, double distance)
{
    CollisionType type;
    for (int i = 0; i < positionListeners.size(); i++)
    {
        type = positionListeners[i]->getCollisionType( getCollisionPosition(i, sensorRange, distance), this );
    }
    return type;
}

geometry_msgs::Point Robot::getCollisionPosition(int index, int sampleSize, double distance)
{
    int angleOffset = (180 - sampleSize) / 2;
    int trueAngle = angleOffset + index;
    double x = distance * cos(trueAngle) + current_x;
    double y = distance * sin(trueAngle) + current_y;
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    return point;
}

void Robot::leftCollisionDetected(CollisionType type)
{
    if (type == Dynamic)
    {
        // Stop moving, wait for obstacle to move
        linear_velocity_x = 0;
        angular_velocity = 0;
    }
    else
    {
        // Spin to the right
        linear_velocity_x =  top_linear_speed;
        angular_velocity = - top_angular_speed;
    }
}

void Robot::rightCollisionDetected(CollisionType type)
{
    if (type == Dynamic)
    {
        // Stop moving, wait for obstacle to move
        linear_velocity_x = 0;
        angular_velocity = 0;
    }
    else
    {
        // Spin to the left
        linear_velocity_x =  top_linear_speed;
        angular_velocity =  top_angular_speed;
    }
}

void Robot::centerCollisionDetected(CollisionType type)
{
    if (type == Dynamic)
    {
        // Stop moving, wait for obstacle to move
        linear_velocity_x = 0;
        angular_velocity = 0;
    }
    else
    {
        // Move backwards and spin right
        linear_velocity_x =  top_linear_speed;
        angular_velocity = - top_angular_speed;
    }
}


void Robot::updateVelocity()
{
    if (current_state == CollisionResolution)
    {
        // Let collision resolution take place before we attempt to move towards the goal
        return;
    }
     if(mycounter>=1 && mycounter<=5){
            mycounter++;
            notifySpeedListeners();
            return;
        }
		else if(mycounter>5 && mycounter<=15){
            if(direction==Left){
                linear_velocity_x =   top_linear_speed;
                angular_velocity =  top_angular_speed;
                ROS_INFO("Left");
            }else if(direction==Right){
                linear_velocity_x =  top_linear_speed;
                 angular_velocity = - top_angular_speed;
                 ROS_INFO("Right");
            }
            notifySpeedListeners();
            mycounter++;
            return;

        }
		else if(mycounter>15){
            mycounter=0;
        }

    // Check if robot has any goals defined. If not, do nothing.
    if (goals.empty())
    {
        //ROS_INFO("No Goal, doing nothing");
        return;
    }

    geometry_msgs::Point desiredLocation = goals[goalIndex];
    // This is the maximum distance a robot can be from it's
    // desired poisition and still be considered to have reached it
    float distanceThreshold = 1.0;
    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - current_x;
    directionVector.y = desiredLocation.y - current_y;

    // Check if we are at the desired location
    if (fabs(directionVector.x) <= distanceThreshold && fabs(directionVector.y) <= distanceThreshold)
    {
        // Robot has reached it's desired location
        linear_velocity_x = 0;
        angular_velocity = 0.0;
        if (goalIndex < goals.size() - 1)
        {
	        reachedCurrentGoal();
        }
        else
        {
            reachedLastGoal();
        }
        
        notifySpeedListeners();
        return;
    }
    
    // Calculate the desired angle
    double desiredAngle = atan2(directionVector.y, directionVector.x);
    rotateToGoal(desiredAngle);

    notifySpeedListeners();
}


void Robot::writeToFile(int id,std::string type,std::string message){

    std::string result = "info/" + type + "/" + type +  std::to_string(id) + ".txt";

    ofstream myfile;
    myfile.open (result,std::ios_base::app);
    myfile << message << "\n";
    myfile.close();

}

void Robot::positionCallback(const nav_msgs::Odometry positionMsg)
{// Handle position data

    // Update Current Position
    geometry_msgs::Pose currentLocation = positionMsg.pose.pose;
    current_x = currentLocation.position.x;
    current_y = currentLocation.position.y;
    double x = currentLocation.orientation.x;
    double y = currentLocation.orientation.y;
    double z = currentLocation.orientation.z;
    double w = currentLocation.orientation.w;
    
    // Calculate orientation
    double yaw = tf::getYaw(tf::Quaternion(x, y, z, w));
    current_theta = yaw;

}

void Robot::notifySpeedListeners()
{// Send current speed to listeners

    // Construct new message
    geometry_msgs::Twist velocityMsg;
    velocityMsg.linear.x = linear_velocity_x;
    velocityMsg.angular.z = angular_velocity;

    // Publish speed message
    positionPub.publish(velocityMsg);

    // Loop through speedListeners and send them this speed message
    for (int i = 0; i < speedListeners.size(); i++)
    {
        speedListeners[i]->speedUpdate(velocityMsg);
    }
    
}

void Robot::reachedCurrentGoal()
{
    goalIndex++;
    ROS_INFO("Reached destination");
    writeToFile(unique_id,robotType,"Reached destination");
}

void Robot::reachedLastGoal()
{
    // Reset index
    ROS_INFO("Reached final destination, going back to the start");
    goalIndex = 0;
    //goalIndex++;
}

void Robot::rotateToGoal(double desiredAngle)
{
    if (fabs(current_theta - desiredAngle) > 0.1)
    {
        // Spin
        current_state = Orienting;
        linear_velocity_x = 0;
        if (current_theta <= desiredAngle)
        {
            angular_velocity = top_angular_speed;
        }
        else
        {
            angular_velocity = -top_angular_speed;
        }

    }
    else
    {
        // Go forward
        current_state = Moving;
        linear_velocity_x = top_linear_speed;
        angular_velocity = 0.0;
    }
}

#endif
