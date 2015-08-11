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
        robotState getState();


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

        int pathIndex;
        geometry_msgs::Point desiredLocations[2];

        // Properties of the physical robot
        double length;
        double width;
        double height;
        double top_speed;

        // The state lets us know what speed to give to the robot
        robotState current_state;
        
    protected:
        // These methods should be overridden in subclasses to provide more specific behavior
        virtual void leftCollisionDetected();
        virtual void rightCollisionDetected();
        virtual void centerCollisionDetected();
        

};

#endif
