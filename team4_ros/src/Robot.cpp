#include "Robot.h"
#include "Util.cpp"


// Contructor
Robot::Robot(double x_position, double y_position, double theta_orientation, int sensor_range, int sensor_angle)
{
    current_x = x_position;
    current_y = y_position;
    current_theta = theta_orientation;
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

    // Setup publisher and subscribers
    positionPub = publisherHandle.advertise<geometry_msgs::Twist>(baseString + "/cmd_vel", 1000);
    groundtruthSub = subscriberHandle.subscribe<nav_msgs::Odometry>(baseString + "/base_pose_ground_truth", 1000, &Robot::positionCallback, this);
    sensorSub = subscriberHandle.subscribe(baseString + "/base_scan", 1000, &Robot::sensorCallback, this);

}

void Robot::addGoal(geometry_msgs::Point goal)
{
    goals.push_back(goal);
}

void Robot::addSpeedListener(SpeedListener* listener)
{
    speedListeners.push_back(listener);
}

robotState Robot::getState()
{
    return this->current_state;
}

void Robot::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorMsg)
{// Handle sensor data
    int left_vals = sensorAngle / 3;
    int right_vals = sensorAngle - left_vals;

    int i = 0;
    bool isNear = false;
    //ROS_INFO("Sensor:");
    for (i; i < sensorAngle; i++) {
        if (sensorMsg->ranges[i] < sensorRange)
        {
            isNear = true;
            current_state = CollisionResolution;

            if (i < left_vals)
            {
                rightCollisionDetected();
                break;
            } else if (i >= left_vals && i < right_vals)
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
    linear_velocity_x = top_linear_speed;
    angular_velocity = -top_angular_speed;
}

void Robot::rightCollisionDetected()
{
    // Spin to the left
    linear_velocity_x = top_linear_speed;
    angular_velocity = top_angular_speed;
}

void Robot::centerCollisionDetected()
{
    // Move backwards and spin right
    linear_velocity_x = 0;
    angular_velocity = -2 * top_angular_speed;
}


void Robot::updateVelocity()
{
    if (current_state == CollisionResolution)
    {
        // Let collision resolution take place before we attempt to move towards the goal
        return;
    }

    // Check if robot has any goals defined. If not, do nothing.
    if (goals.empty())
    {
        ROS_INFO("No Goal, doing nothing");
        return;
    }

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
        angular_velocity = 0.0;
        if (goalIndex < goals.size() - 1)
        {
            goalIndex++;
            ROS_INFO("Reached destination");
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
    velocityMsg.angular.z = angular_velocity;

    // Publish speed message
    positionPub.publish(velocityMsg);

    // Loop through speedListeners and send them this speed message
    for (int i = 0; i < speedListeners.size(); i++)
    {
        speedListeners[i]->speedUpdate(velocityMsg);
    }
    
}

void Robot::reachedLastGoal()
{
    // Reset index
    ROS_INFO("Reached final destination, going back to the start");
    goalIndex++;
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


