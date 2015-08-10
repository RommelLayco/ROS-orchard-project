#include "ros/ros.h"
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <sstream>
#include "math.h"
#include <unistd.h>
#include <vector>
#include "SpeedListener.h"


enum robotState {CollisionResolution, Moving, Orienting};


class Robot
{
    public:
        Robot(double x, double y, double z);
        void updateVelocity();
        void addSpeedListener(SpeedListener* listener);
        void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorMsg);
        void positionCallback(const nav_msgs::Odometry PositionMsg);
        void addGoal(geometry_msgs::Point goal);


    private:

        void notifySpeedListeners(); // Send position message to all listeners

        std::vector<SpeedListener*> speedListeners; // Must be a pointer because SpeedListener is an abstract type

        std::vector<geometry_msgs::Point> goals;
        int goalIndex;

        // The attributes of the robot
        double linear_velocity_x;
        double linear_velocity_y;
        double angular_veloctiy;
        double current_x;
        double current_y;
        double current_theta;

        //////////////
        int pathIndex;
        geometry_msgs::Point desiredLocations[2];
        //////////////

        // Properties of the physical robot
        double length;
        double width;
        double height;
        double top_speed;

        // The state lets us know what speed to give to the robot
        robotState current_state;

};



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
            ROS_INFO("I'm near something! [%f]", sensorMsg->ranges[i]);

            if (i < 60)
            {
                // Spin to the left
                ROS_INFO("Spinning left");
                linear_velocity_x = 1;
                angular_veloctiy = 0.5;
                break;
            } else if (i >= 60 && i < 120)
            {
                // Move backwards and spin right
                ROS_INFO("Moving backwards and spinning right");
                linear_velocity_x = 0;
                angular_veloctiy = -1.0;
                break;
            } else
            {
                // Spin to the right
                ROS_INFO("Spinning right");
                linear_velocity_x = 1;
                angular_veloctiy = -0.5;
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

    ROS_INFO("X distance: [%f]", current_x);
    ROS_INFO("Y distance: [%f]", current_y);

    // Check if we are at the desired location
    if (abs(directionVector.x) <= distanceThreshold && abs(directionVector.y) <= distanceThreshold)
    {
        // Robot has reached it's desired location
        // For now, make robot stop. In future, robot should now try to move
        // to the next location on it's path.
        ROS_INFO("I have reached my destination!");
        linear_velocity_x = 0;
        angular_veloctiy = 0.0;
        if (goalIndex < goals.size() - 1)
        {
            goalIndex++;
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

    ROS_INFO("Theta = [%f]", current_theta);

    // If the desired angle is 0
    if(desiredAngle != 0 && desiredAngle != M_PI)
    {    

        // If the difference between current angle and desired angle is less than 0.1 stop spining
        if (abs(current_theta - desiredAngle) > 0.1)
        {
            // Spin
            current_state = Orienting;
            ROS_INFO("Spinning!");
            linear_velocity_x = 0;
            if (current_theta <= desiredAngle)
            {
                angular_veloctiy = 0.5;
            }
            else
            {
                angular_veloctiy = -0.5;
            }
            
        } else
        {
            // Go forward
            current_state = Moving;
            linear_velocity_x = 1;
            angular_veloctiy = 0;
        }
    }
    notifySpeedListeners();
}


void Robot::positionCallback(const nav_msgs::Odometry positionMsg)
{// Handle position data

    //ROS_INFO("X distance: [%f]", positionMsg.pose.pose.position.x);
    //ROS_INFO("Y distance: [%f]", positionMsg.pose.pose.position.y);

    // Update Current Position
    geometry_msgs::Pose currentLocation = positionMsg.pose.pose;
    current_x = currentLocation.position.x;
    current_y = currentLocation.position.y;
    double z = currentLocation.orientation.z;
    double w = currentLocation.orientation.w;
    
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(current_x, current_y, z, w)).getRPY(roll, pitch, yaw);
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



class myClass: public SpeedListener {
    private:
        ros::Publisher publisher;
    public:
        myClass(ros::Publisher pub);
        void speedUpdate(geometry_msgs::Twist speedMsg);
};

myClass::myClass(ros::Publisher pub)
{
    publisher = pub;
}

void myClass::speedUpdate(geometry_msgs::Twist speedMsg){
    publisher.publish(speedMsg);
}




int main(int argc, char **argv)
{
    // Create robot object
    Robot myRobot = Robot(0, 0, 0);

    ros::init(argc, argv, "Robot");

    // Setup ros handles
    ros::NodeHandle publisherHandle;
    ros::NodeHandle subscriberHandle;

    ros::Publisher mypub_object = publisherHandle.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);

    // Attach publisher to robot object
    myClass temp = myClass(mypub_object);
    myRobot.addSpeedListener(&temp);

    // loop 10 Hz
	ros::Rate loop_rate(10);

    ros::Subscriber groundtruthSub = subscriberHandle.subscribe<nav_msgs::Odometry>("robot_0/base_pose_ground_truth",1000, &Robot::positionCallback, &myRobot);
    ros::Subscriber SensorSub = subscriberHandle.subscribe("robot_0/base_scan", 1000, &Robot::sensorCallback, &myRobot);


    // Add some goals to robot
    geometry_msgs::Point desiredLocation1;
    desiredLocation1.x = 1.6;
    desiredLocation1.y = -20;
    desiredLocation1.z = 0;

    geometry_msgs::Point desiredLocation2;
    desiredLocation2.x = 1.6;
    desiredLocation2.y = -2;
    desiredLocation2.z = 0;


    myRobot.addGoal(desiredLocation1);
    myRobot.addGoal(desiredLocation2);



    while (ros::ok()) 
	{ 
        // In future, will loop through all robot objects calling this method on each of them
		myRobot.updateVelocity();

		ros::spinOnce();
		loop_rate.sleep();
	} 

	return 0; 



}



