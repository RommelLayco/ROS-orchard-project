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
        /*virtual void reachedLastGoal();*/
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
    // Spin to the right
    ROS_INFO("Dog Left collision");
    linear_velocity_x = 4 * top_linear_speed;
    angular_velocity = -4 * top_angular_speed;
}

void Dog::rightCollisionDetected()
{
    ROS_INFO("Dog Right collision");
    // Spin to the left
    linear_velocity_x = 4 * top_linear_speed;
    angular_velocity = 4 * top_angular_speed;
}

void Dog::centerCollisionDetected()
{
        ROS_INFO("Dog Center collision");
    // Move backwards and spin right
    linear_velocity_x = 4 * top_linear_speed;
    angular_velocity = -4 * top_angular_speed;
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

/*void Dog::reachedLastGoal()
{
    //generateRandomDesiredLocations();

}*/

