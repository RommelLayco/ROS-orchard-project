#include "Robot.h"
#include <time.h> 


class Person: public Robot
{
    public:
        Person(double x, double y, double z) : Robot(x, y, z) {}
    protected:
        virtual void leftCollisionDetected();
        virtual void rightCollisionDetected();
        virtual void centerCollisionDetected();
        void generateRandomDesiredLocations();
        virtual void reachedLastGoal();
};


void Person::leftCollisionDetected()
{
    linear_velocity_x = -top_linear_speed;
    angular_velocity = - 2 * top_angular_speed;
}

void Person::rightCollisionDetected()
{
    ROS_INFO("SUBCLASS!");
}

void Person::centerCollisionDetected()
{
    ROS_INFO("SUBCLASS!");
}

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

