#ifndef ROBOT_H
#define ROBOT_H

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
#include "Robot.h"
#include "PositionListener.h"
#include "Util.cpp"

/* Represents the three possible states an entity can be in */
enum robotState {CollisionResolution, Moving, Orienting};


/* Base class that represents an entity in the simulation */
class Robot
{
    public:
        Robot(double x, double y, double z, int sensor_range, int sensor_angle);
        void updateVelocity();
        void addSpeedListener(SpeedListener* listener);
        void addPositionListener(PositionListener* listener);
        void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorMsg);
        void positionCallback(const nav_msgs::Odometry PositionMsg);
        void addGoal(geometry_msgs::Point goal);
        robotState getState();
        double getXPos();
        double getYPos();

        void writeToFile(std::string message);


   protected:
        void notifySpeedListeners(); // Send position message to all listeners

        std::vector<SpeedListener*> speedListeners; // Must be a pointer because SpeedListener is an abstract type
        std::vector<PositionListener*> positionListeners;

        geometry_msgs::Point getCollisionPosition(int index, int sampleSize, double distance);

        CollisionType getCollisionType(int i, int sensorRange, double distance);

        // List of entity's goals
        std::vector<geometry_msgs::Point> goals;
        int goalIndex;

        // Publisher and subscriber objects
        ros::Publisher positionPub;
        ros::Subscriber groundtruthSub;
        ros::Subscriber sensorSub;

        // Ros handles
        ros::NodeHandle publisherHandle;
        ros::NodeHandle subscriberHandle;

        // The attributes of the robot
        double linear_velocity_x;
        double linear_velocity_y;
        double angular_velocity;
        double current_x;
        double current_y;
        double current_theta;

        int pathIndex;
        geometry_msgs::Point desiredLocations[2];

        // Properties of the physical robot
        double length;
        double width;
        double height;
        double top_linear_speed = 1.0;
        double top_angular_speed = 0.5;
        int sensorAngle; // This must correspond to sensor value defined in the world file
        double sensorRange; // This determines the range (in meters) at which the entity will detect and object

        // The state lets us know what speed to give to the robot
        robotState current_state;

        // Method that is called from updateVelocity() when entity wishes to rotate towards it's current goal
        void rotateToGoal(double desiredAngle);
    
        // These methods should be overridden in subclasses to provide more specific behavior
        virtual void leftCollisionDetected(CollisionType type);
        virtual void rightCollisionDetected(CollisionType type);
        virtual void centerCollisionDetected(CollisionType type);
        // Subclass should overide this to define behavior when last goal is reached
        virtual void reachedLastGoal();
        

};

#endif
