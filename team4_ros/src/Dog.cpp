#include "Robot.h"
#include <time.h>
#include "std_msgs/String.h"


class Dog: public Robot
{
    public:
        Dog(int sensor_range, int sensor_angle, int number, std::string type) : Robot(sensor_range, sensor_angle,number,type)
        {
            // Setup bark publisher
            barkPub = publisherHandle.advertise<std_msgs::String>("dog_topic", 1000);
        }
    protected:
        ros::Publisher barkPub;
        virtual void leftCollisionDetected(CollisionType type);
        virtual void rightCollisionDetected(CollisionType type);
        virtual void centerCollisionDetected(CollisionType type);
        void generateRandomDesiredLocations();
        void bark();
};

/* Publish dog's bark message */
void Dog::bark()
{
    std_msgs::String barkMsg;
    barkMsg.data = "I AM BARKING!! Woof Woof!!";
    barkPub.publish(barkMsg);

    writeToFile(unique_id,robotType, "I AM BARKING!! Woof Woof!!");
}

void Dog::leftCollisionDetected(CollisionType type)
{
    // Spin to the right
    linear_velocity_x = 4 * top_linear_speed;
    angular_velocity = -4 * top_angular_speed;
}

void Dog::rightCollisionDetected(CollisionType type)
{
    // Spin to the left
    linear_velocity_x = 4 * top_linear_speed;
    angular_velocity = 4 * top_angular_speed;
}

void Dog::centerCollisionDetected(CollisionType type)
{
    // Move forwards and spin right
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

