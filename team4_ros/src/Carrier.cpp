#include "Robot.h"
#include "Bin.cpp"
#include <team4_ros/binIsFull.h>

enum CarrierState {MovingToBin, ShiftingBin, MovingToInitial, Idle};
class Carrier: public Robot
{
    public:
        Carrier(int sensor_range, int sensor_angle, int number, std::string type, geometry_msgs::Point binDropOffLocation);
        bool hasBin();
        bool moveToBin(geometry_msgs::Point binLocation);
    protected:
        Bin* bin;
        CarrierState state;
        geometry_msgs::Point binDropOffLocation;
        std::vector<geometry_msgs::Point> binGoals; // Locations of bins that carrier should move to drop off
        int binGoalIndex;

        bool pickupBin(Bin* b);
        void deliverBin();
        void dropBin();
        virtual void reachedCurrentGoal();
        virtual void reachedLastGoal();
};

Carrier::Carrier(int sensor_range, int sensor_angle, int number, std::string type, geometry_msgs::Point binDropOff)
    : Robot(sensor_range, sensor_angle, number, type)
    {
        // Setup Bin full publisher
        bin = NULL;
        binDropOffLocation = binDropOff;
        state = Idle;
    }

bool Carrier::moveToBin(geometry_msgs::Point binLocation)
{
    // Add bin location to list
    //binGoals.push_back(binLocation);
    goals.push_back(binLocation);
    state = MovingToBin;
    ROS_INFO("Moving to bin");

}

/* If carrier already has bin, return false */
bool Carrier::pickupBin(Bin* b)
{
    // TODO check that carrier is actually at bin
    if (bin == NULL)
    {
        bin = b;
        state = ShiftingBin;
        return true;
    }
    else
    {
        // Carrier already has bin
        return false;
    }
}

/* If carrier has a bin, move it to dropoff point */
void Carrier::deliverBin()
{
    // Check if carrier has bin
    if (bin != NULL)
    {
        goals.push_back(binDropOffLocation);
    }
}

void Carrier::dropBin()
{
    // TODO Implement this

    bin = NULL;
    state = MovingToInitial;
}

void Carrier::reachedCurrentGoal()
{
    if (state == MovingToBin)
    {
        // Has reached bin it was moving to, now pick it up
        deliverBin();
    }
    else if (state == MovingToInitial)
    {
        // Reached inital position
        state == Idle;
    }
    else if (state == ShiftingBin)
    {
        // Reached drop off point.
        dropBin();
    }

}


void Carrier::reachedLastGoal()
{
    // When carrier has no more goals, it should move back to it's start goal and wait
    // If carrier recieves new goal while it is heading back to home, it should switch
    // to this new goal.

}

