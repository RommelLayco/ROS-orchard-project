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

enum collisionDirection {Left,Right};

/* Base class that represents an entity in the simulation */
class Robot
{
public:
    Robot(int sensor_range, int sensor_angle, int id, std::string type);

       /**
        * This method is invoked by MainNode in a loop to progress the simulation.
        * It is here that the entity performs calculations to determine its correct
        * speed and orientation.
        */
        void updateVelocity();

       /**
        * Subscribes a speed listener to this entity, so that the listener will recieve
        * updates on the entity's speed.
        *
        * @param listener  the address of the SpeedListener to add as subscriber
        */
        void addSpeedListener(SpeedListener* listener);

       /**
        * Subscribes a position listener to this entity. When the entity detects a collision,
        * it will callback to position listeners with its position in order to determine what
        * it is colliding with.
        *
        * @param listener  the address of the PositionListener to add
        */
        void addPositionListener(PositionListener* listener);

       /**
        * This method is invoked by ROS when sensor data is available for this entity.
        *
        * @param sensorMsg  sensor data
        */
        void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sensorMsg);

       /**
        * This method is invoked by ROS when position data for this entity is available.
        *
        * @param sensorMsg  sensor data
        */
        void positionCallback(const nav_msgs::Odometry PositionMsg);

       /**
        * Add a goal (as x, y coordinates) that the entity should navigate to.
        *
        * @param goal  goal to append to entity's list
        */
        void addGoal(geometry_msgs::Point goal);

       /**
        * Get the current state of the entity.
        *
        * @return  enum representing the current state of the entity
        */
        robotState getState();

       /**
        * @return  x coordinate of entity
        */
        double getXPos();

       /**
        * @return  y coordinate of entity
        */
        double getYPos();

       /**
        * Write a string to a file. Used for debugging.
        * @param message  string that should be written out to a file
        */
        void writeToFile(int id,std::string type, std::string message);

	/**
        * Write a string for inital direction of picker to a file. Used for debugging.
        * @param message  string that should be written out to a file
        */
	void pickerInitWrite();

	/**
        * Write a string for tractor to a file. Used for debugging.
        * @param message  string that should be written out to a file
        */
	void tractorWrite();

	/**
        * Write a string for tractor to a file. Used for debugging.
        * @param message  string that should be written out to a file
        */
	void animalWrite();

	/**
        * Write a string for person to a file. Used for debugging.
        * @param message  string that should be written out to a file
        */
	void humanWrite();

        /**
        * this integer is unique for each robit
        *
        */
        int unique_id;

        /* this string is set via the constructor. It is for making a  file*/
        std::string robotType;


    protected:
        std::vector<SpeedListener*> speedListeners; // Must be a pointer because SpeedListener is an abstract type
        std::vector<PositionListener*> positionListeners;

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
        double top_linear_speed = 1.0;
        double top_angular_speed = 0.5;
        int sensorAngle; // This must correspond to sensor value defined in the world file
        double sensorRange; // This determines the range (in meters) at which the entity will detect an object

        // The state lets us know what speed to give to the robot
        robotState current_state;

        // Counter for overrall collison avoid
        int mycounter;
        int stopCounter;

        collisionDirection direction;


       /**
        * Send a position message to all speed listeners.
        */
        void notifySpeedListeners();

       /**
        * Called when entity detects it is about to collide with something.
        * This method asks the main node to determine whether the collision is
        * with a static object e.g a tree or a dynamic object e.g a Person or picker.
        *
        * @param sensorIndex  the index in the sensor array where the collision was detected
        * @param sampleSize   the total number of samples in the sensor data
        * @param distance     the distance from the entity the collision was detected
        * @return             type of object colliding with (Static or Dynamic)
        */
        CollisionType getCollisionType(int sensorIndex, int sensorRange, double distance);

       /**
        * Method that is called from updateVelocity() when entity wishes to rotate towards it's current goal.
        *
        * @param desiredAngle  the angle in radians that the entity should rotate through
        */
        void rotateToGoal(double desiredAngle);

       /**
        * Called from sensorCallback() when entity detects a collision on its left.
        * Should be overridden in subclasses to provide more specific behavior.
        *
        * @param CollisionType  the type of collision (Static or Dynamic)
        */
        virtual void leftCollisionDetected(CollisionType type);

       /**
        * Called from sensorCallback() when entity detects a collision on its right.
        * Should be overridden in subclasses to provide more specific behavior.
        *
        * @param CollisionType  the type of collision (Static or Dynamic)
        */
        virtual void rightCollisionDetected(CollisionType type);

       /**
        * Called from sensorCallback() when entity detects a collision directly in front of it.
        * Should be overridden in subclasses to provide more specific behavior.
        *
        * @param CollisionType  the type of collision (Static or Dynamic)
        */
        virtual void centerCollisionDetected(CollisionType type);

       /**
        * Called from updateVelocity() when entity reaches its current goal.
        * Should be overridden in subclasses to provide more specific behavior.
        */
	virtual void reachedCurrentGoal();

       /**
        * Called from updateVelocity() when entity reaches its final goal.
        * Should be overridden in subclasses to provide more specific behavior.
        */
        virtual void reachedLastGoal();

       /**
        * Called from sensorCallback() when entity detects a collision directly in front of it.
        * Should be overridden in subclasses to provide more specific behavior.
        *
        * @param index       the index in the sensor array where the collision was detected
        * @param sampleSize  the total number of samples in the sensor data
        * @param distance    the distance from the entity the collision was detected
        * @return            point corresponding to x, y coordinates of collision location
        */
        geometry_msgs::Point getCollisionPosition(int index, int sampleSize, double distance);

    };

#endif
