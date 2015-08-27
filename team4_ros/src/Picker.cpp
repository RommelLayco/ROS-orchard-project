#include "Robot.h"
#include <team4_ros/binIsFull.h>
#include "Bin.cpp"

class Picker: public Robot
{
public:
    Picker(int sensor_range, int sensor_angle, int number, std::string type);
    void pickupBin(Bin* bBin);
protected:
    Bin* bin;
    ros::Timer timer;
    // Publisher object for picker
    ros::Publisher binFullPub;
    ros::NodeHandle timerHandle;
    virtual void reachedLastGoal();
    void binIsFull();
    void timerCallback(const ros::TimerEvent&);
};

Picker::Picker(int sensor_range, int sensor_angle, int number, std::string type)
    : Robot(sensor_range, sensor_angle, number, type)
{
    // Set up Bin full publisher
    binFullPub = publisherHandle.advertise<team4_ros::binIsFull>("bin_topic", 1000);
    bin = NULL;
    timer = timerHandle.createTimer(ros::Duration(2.0), &Picker::timerCallback, this);

}

/* Called by Picker when its bin is full */
void Picker::binIsFull()
{
    if (bin == NULL)
    {
        return;
    }
    team4_ros::binIsFull binMsg;
    binMsg.isFull = true;
    // Set x and y coordinates of bin
    binMsg.x = current_x;
    binMsg.y = current_y;
    // Publish binIsFull message
    binFullPub.publish(binMsg);

    ROS_INFO("Publish is called");

    bin = NULL;
    ROS_INFO("Picker[%d] Dropped Bin", unique_id);
    // Create new bin
    Bin* b = new Bin(0, 0, 20);
    bin = b;
}

void Picker::pickupBin(Bin* b)
{
    bin = b;
    ROS_INFO("Picker [%d] Picked up Bin", unique_id);
}

void Picker::reachedLastGoal()
{
    //binIsFull();
    goalIndex = 0;
}

void Picker::timerCallback(const ros::TimerEvent&)
{
    if (current_state == Moving)
    {
        // Pick fruit
        if (bin != NULL && ! bin->isFull())
        {
            ROS_INFO("Picker [%d] Picked some fruit", unique_id);
            bin->addItem();
        }
        else if (bin != NULL && bin->isFull())
        {
            // Bin is full. Drop it and notify a carrier
            binIsFull();
        }
        else
        {
            // Picker has no bin, do nothing
            linear_velocity_x = 0;
            angular_velocity = 0;
            notifySpeedListeners();
        }
    }
}

