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


    mycounter = 0;
    stopCounter = 0;
    direction = Left;

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
    goals.push_back(goal);
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
{
    // Handle sensor data
    double left_vals = sensorAngle / 2;
    double right_vals = sensorAngle - left_vals;

    double i = 0;
    bool isNear = false;
    // Loop through sensor data array
    for (i; i < sensorAngle; i++)
    {
        // If an object is detected within sensorRange,
        // determine it's position relative to the entity
        if (sensorMsg->ranges[i] < sensorRange)
        {
            // Write to debugger that it near an obstacle for picker
            if (robotType == "picker")
            {
                writeToFile(unique_id, robotType, "Near an obstacle");
            }

            isNear = true;
            mycounter=1;
            current_state = CollisionResolution; // Entity is now in CollisionResolution state
            CollisionType type = getCollisionType(i, sensorRange, sensorMsg->ranges[i]);
            if ( i < 5)
            {
                linear_velocity_x = top_linear_speed;
                angular_velocity = top_angular_speed;
                notifySpeedListeners();
                return;
            }

            if (i < left_vals)
            {
                // Collision is on right
                direction = Right;
                rightCollisionDetected(type);
                return;
            }
            else if (i >= left_vals && i < sensorAngle)
            {
                // Collision is in front
                direction = Left;
                centerCollisionDetected(type);
                return;
            }
            else
            {
                // Collision is on left
                direction = Left;
                leftCollisionDetected(type);
                return;
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
        linear_velocity_x = top_linear_speed;
        angular_velocity = -top_angular_speed;
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
        linear_velocity_x = top_linear_speed;
        angular_velocity = top_angular_speed;
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
        linear_velocity_x = top_linear_speed;
        angular_velocity = -top_angular_speed;
    }
}


void Robot::updateVelocity()
{
    if (linear_velocity_x == 0 && angular_velocity == 0)
    {
        stopCounter++;
    }

    if (stopCounter >= 10 && stopCounter < 15)
    {
        angular_velocity = 0.1;
        linear_velocity_x = 0;
        stopCounter++;
        notifySpeedListeners();
        return;
    }
    else if (stopCounter >= 15)
    {
        stopCounter = 0;
    }

    if (current_state == CollisionResolution)
    {
        // Let collision resolution take place before we attempt to move towards the goal
        return;
    }

    if (mycounter >= 1 && mycounter <= 5)
    {
        mycounter++;
        notifySpeedListeners();
        return;
    }
    else if (mycounter > 5 && mycounter <= 15)
    {
        if (direction == Left)
        {
            linear_velocity_x = top_linear_speed;
            angular_velocity = top_angular_speed;
        }
        else if (direction == Right)
        {
            linear_velocity_x = top_linear_speed;
            angular_velocity = -top_angular_speed;
        }

        notifySpeedListeners();
        mycounter++;
        return;

    }
    else if (mycounter > 15)
    {
        mycounter = 0;
    }

    // Check if robot has any goals defined. If not, do nothing.
    if (goals.empty())
    {
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


void Robot::writeToFile(int id, std::string type, std::string message)
{

    std::string result = "info/" + type + "/" + type +  std::to_string(id) + ".txt";

    ofstream myfile;
    myfile.open (result, std::ios_base::app);
    myfile << message << "\n";
    myfile.close();

}

void Robot::positionCallback(const nav_msgs::Odometry positionMsg)
{
    // Handle position data

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
{
    // Send current speed to listeners

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

    // Write to file next destination
    if (robotType == "picker") // If picker, give coordinates of next destination
    {
        if (goalIndex == 0)
        {
            writeToFile(unique_id, robotType, "Reached destination, now moving down the orchard, away from the orchard");
        }
        else
        {
            writeToFile(unique_id, robotType, "Reached destination, now moving up the orchard, towards the driveway");
        }
    }
    else if (robotType == "tractor")
    {
        tractorWrite();
    }
    else if (robotType == "animal")
    {
        animalWrite();
    }
    else if (robotType == "human")
    {
        humanWrite();
    }
    else
    {
        // Do nothing
    }
}

void Robot::reachedLastGoal()
{
    // Reset index
    ROS_INFO("Reached final destination, going back to the start");
    goalIndex = 0;
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

void Robot::tractorWrite()
{
    // Check goal index

    if (goalIndex == 0)
    {
        writeToFile(unique_id, robotType, "Reach destination, Now going to the top left corner");
    }
    else if (goalIndex == 1)
    {
        writeToFile(unique_id, robotType, "Reach destination, Now going to the bottom left corner");
    }
    else if (goalIndex == 2)
    {
        writeToFile(unique_id, robotType, "Reach destination, Now going to the bottom right corner");
    }
    else if (goalIndex == 3)
    {
        writeToFile(unique_id, robotType, "Reach destination, Now going to the top right corner near the driveway");
    }
    else
    {
        // Do nothing
        writeToFile(unique_id, robotType, "Error");
    }
}

void Robot::animalWrite()
{
    // Check goal index

    if (goalIndex % 4 == 0)
    {
        // Read in current goal
        geometry_msgs::Point desiredLocation = goals[goalIndex];
        double x = desiredLocation.x + 1.2;
        double y = desiredLocation.y;

        std::string line = "Reached destination, now going to tree located at: ";
        std::string result = line + std::to_string (x) + "," + std::to_string (y);

        writeToFile(unique_id, robotType, result);
    }

}

void Robot::pickerInitWrite()
{
    std::string line = "Picker moving up the orchard towards the driveway ";
    writeToFile(unique_id, robotType, line);
}

void Robot::humanWrite()
{
    // Read in current goal
    geometry_msgs::Point desiredLocation = goals[goalIndex];
    double x = desiredLocation.x;
    double y = desiredLocation.y;

    std::string line = "Reached destination, Checking area around: ";
    std::string result = line + std::to_string (x) + "," + std::to_string (y);

    writeToFile(unique_id, robotType, result);

}

