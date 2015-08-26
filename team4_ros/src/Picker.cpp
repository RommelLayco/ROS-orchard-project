#include "Robot.h"
#include <team4_ros/binIsFull.h>


class Picker: public Robot
{
    public:
        Picker(int sensor_range, int sensor_angle, int number, std::string type);
    protected:
        // TODO add bin field
        // Publisher object for picker
        ros::Publisher binFullPub;

        virtual void reachedLastGoal();
        void binIsFull();
};

Picker::Picker(int sensor_range, int sensor_angle, int number, std::string type)
    : Robot(sensor_range, sensor_angle, number, type)
    {
        // Set up Bin full publisher
        binFullPub = publisherHandle.advertise<team4_ros::binIsFull>("bin_topic", 1000);
    }

/* Called by Picker when its bin is full */
void Picker::binIsFull()
{
    team4_ros::binIsFull binMsg;
    binMsg.isFull = true;
    // Set x and y coordinates of bin
    binMsg.x = current_x;
    binMsg.y = current_y;
    // Publish binIsFull message
    binFullPub.publish(binMsg);
}


void Picker::reachedLastGoal()
{
    binIsFull();
}

