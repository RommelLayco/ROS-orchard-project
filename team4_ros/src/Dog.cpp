#include "Robot.h"
#include <time.h>
#include "std_msgs/String.h"


class Dog: public Robot
{
    public:
        Dog(double x, double y, double z, int sensor_range, int sensor_angle) : Robot(x, y, z, sensor_range, sensor_angle)
        {
            // Setup bark publisher
            barkPub = publisherHandle.advertise<std_msgs::String>("dog_topic", 1000);
        }
    protected:
        ros::Publisher barkPub;
        virtual void leftCollisionDetected();
        virtual void rightCollisionDetected();
        virtual void centerCollisionDetected();
        void generateRandomDesiredLocations();
        virtual void reachedLastGoal();
        void bark();
};

/* Publish dog's bark message */
void Dog::bark()
{
    std_msgs::String barkMsg;
    barkMsg.data = "I AM BARKING!! Woof Woof!!";
    barkPub.publish(barkMsg);
}

void Dog::leftCollisionDetected()
{
    // Move back and spin clockwise
    ROS_INFO("LEFT COLLISION");
    linear_velocity_x = 1.0;
    angular_velocity = -0.5;
}

void Dog::rightCollisionDetected()
{
    // Move back and spin anticlockwise
    ROS_INFO("RIGHT COLLISION");
    linear_velocity_x = 1.0;
    angular_velocity = 0.5;
}

void Dog::centerCollisionDetected()
{
    bark();
    // Move back faster, obstacle at middle.
    ROS_INFO("CENTER COLLISION");
    linear_velocity_x = 0.0;
    angular_velocity = -1.0;
}

void Dog::generateRandomDesiredLocations()
{
    // set up for random number generation
    srand (time(NULL));

    // Setup points on robot's path
    geometry_msgs::Point desiredLocation1;
    desiredLocation1.x = rand() % 10 + 1;
    desiredLocation1.y = rand() % 10 + 1;
    desiredLocation1.z = 0;

    geometry_msgs::Point desiredLocation2;
    desiredLocation2.x = rand() % 10 + 1;
    desiredLocation2.y = -1 * (rand() % 10 + 1);
    desiredLocation2.z = 0;

    addGoal(desiredLocation1);
    addGoal(desiredLocation2);
}

void Dog::reachedLastGoal()
{
    generateRandomDesiredLocations();
}

