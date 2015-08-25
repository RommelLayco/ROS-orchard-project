#include "Robot.h"
#include <time.h> 


class Person: public Robot
{
    public:
        Person(int sensor_range, int sensor_angle,int number, std::string type) : Robot(sensor_range, sensor_angle,number,type) {}
    protected:
        /*virtual void leftCollisionDetected();
        virtual void rightCollisionDetected();
        virtual void centerCollisionDetected();*/
        void generateRandomDesiredLocations();
        virtual void reachedLastGoal();
};


/*void Person::leftCollisionDetected()
{
    // Move back and spin clockwise
    ROS_INFO("LEFT COLLISION");
    linear_velocity_x = 1.0;
    angular_velocity = -0.5;
}

void Person::rightCollisionDetected()
{
    // Move back and spin anticlockwise
    ROS_INFO("RIGHT COLLISION");
    linear_velocity_x = 1.0;
    angular_velocity = 0.5;
}

void Person::centerCollisionDetected()
{
    // Move back faster, obstacle at middle.
    ROS_INFO("CENTER COLLISION");
    linear_velocity_x = 0.0;
    angular_velocity = -2.0;
}*/

void Person::generateRandomDesiredLocations()
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

void Person::reachedLastGoal()
{
    generateRandomDesiredLocations();
}

