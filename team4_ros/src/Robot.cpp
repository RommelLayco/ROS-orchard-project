#include "Robot.h"


// Contructor
Robot::Robot(double x_position, double y_position, double theta_orientation)
{
    current_x = x_position;
    current_y = y_position;
    current_theta = theta_orientation;
    linear_velocity_x = 0;
    linear_velocity_y = 0;
    angular_veloctiy = 0;
    current_state = Orienting;
    goalIndex = 0;

}

void Robot::addGoal(geometry_msgs::Point goal)
{
    goals.push_back(goal);
}

void Robot::addSpeedListener(SpeedListener* listener)
{
    speedListeners.push_back(listener);
}


void Robot::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorMsg)
{// Handle sensor data

    int i = 0;
    bool isNear = false;
    //ROS_INFO("Sensor:");
    for (i; i < 180; i++) {
        if (sensorMsg->ranges[i] < 1.5)
        {
            isNear = true;
            current_state = CollisionResolution;
            //ROS_INFO("I'm near something! [%f]", sensorMsg->ranges[i]);

            if (i < 60)
            {
                rightCollisionDetected();
                break;
            } else if (i >= 60 && i < 120)
            {
                centerCollisionDetected();
                break;
            } else
            {
                leftCollisionDetected();
                break;
            }

        }

        if (isNear == false)
        {
            current_state = Orienting;
        }
        
        notifySpeedListeners();

    }
}


void Robot::leftCollisionDetected()
{
    // Spin to the right
    //ROS_INFO("Spinning right");
    linear_velocity_x = 1;
    angular_veloctiy = -0.5;
}

void Robot::rightCollisionDetected()
{
    // Spin to the left
    //ROS_INFO("Spinning left");
    linear_velocity_x = 1;
    angular_veloctiy = 0.5;
}

void Robot::centerCollisionDetected()
{
    // Move backwards and spin right
    //ROS_INFO("Moving backwards and spinning right");
    linear_velocity_x = 0;
    angular_veloctiy = -1.0;
}


void Robot::updateVelocity() {

    if (current_state == CollisionResolution)
    {
        // Let collision resolution take place before we attempt to move towards the goal
        return;
    }

    // Find the correct angle

    geometry_msgs::Point desiredLocation = goals[goalIndex];
    // This is the maximum distance a robot can be from it's
    // desired poisition and still be considered to have reached it
    float distanceThreshold = 0.5;

    geometry_msgs::Point directionVector; // Vector from currentLocation to desiredLocation
    directionVector.x = desiredLocation.x - current_x;
    directionVector.y = desiredLocation.y - current_y;

    // Check if we are at the desired location
    if (abs(directionVector.x) <= distanceThreshold && abs(directionVector.y) <= distanceThreshold)
    {
        // Robot has reached it's desired location
        // For now, make robot stop. In future, robot should now try to move
        // to the next location on it's path.
        linear_velocity_x = 0;
        angular_veloctiy = 0.0;
        if (goalIndex < goals.size() - 1)
        {
            goalIndex++;
            ROS_INFO("Reached destination");
        }
        else
        {
            // Reset index
            ROS_INFO("Reached final destination, going back to the start");
            goalIndex = 0;
        }
        
        notifySpeedListeners();
        return;
    }
    
    // Calculate the desired angle
    double desiredAngle = atan2(directionVector.y, directionVector.x);

    if (fabs(current_theta - desiredAngle) > 0.1)
    {
        // Spin
        current_state = Orienting;
        //ROS_INFO("Spinning!");
        linear_velocity_x = 0;
        if (current_theta <= desiredAngle)
        {
            angular_veloctiy = 0.5;
        }
        else
        {
            angular_veloctiy = -0.5;
        }

    }
    else
    {
        // Go forward
        current_state = Moving;
        //ROS_INFO("Moving!");
        linear_velocity_x = 1;
        angular_veloctiy = 0;
    }

    notifySpeedListeners();
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
    
    double yaw = tf::getYaw(tf::Quaternion(x, y, z, w));
    current_theta = yaw;
    
}


void Robot::notifySpeedListeners()
{// Send current speed to listeners

    // Construct new message
    geometry_msgs::Twist velocityMsg;
    velocityMsg.linear.x = linear_velocity_x;
    velocityMsg.angular.z = angular_veloctiy;

    // Loop through speedListeners and send them this speed message
    for (int i=0; i < speedListeners.size(); i++)
    {
        speedListeners[i]->speedUpdate(velocityMsg);
    }

    
}

