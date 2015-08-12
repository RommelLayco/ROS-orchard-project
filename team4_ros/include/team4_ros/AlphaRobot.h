#ifndef ALPHAROBOT_H
#define ALPHAROBOT_H


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

class AlphaRobot {
    public:               
        void groundTruthCallback(const nav_msgs::Odometry msg) ;
	void updateCurrentVelocity();
    };
#endif
