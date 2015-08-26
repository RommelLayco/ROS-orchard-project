#include "Robot.h"
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
        geometry_msgs::Point home;
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
	
	//write to debugger
	writeToFile(unique_id,robotType,"Waiting for a full bin");
    }

bool Carrier::moveToBin(geometry_msgs::Point binLocation)
{
    // Add bin location to list
    binGoals.push_back(binLocation);
    if (state == Idle)
    {
        goals.push_back(binLocation);
        state = MovingToBin;

	//write bin location
	double x = binLocation.x;
	double y = binLocation.y;	

	std::string line = "Moving to a full bin at: ";
	std::string result = line + std::to_string (x) + "," + std::to_string (y);
	writeToFile(unique_id,robotType,result);
    }

}

/* If carrier already has bin, return false */
bool Carrier::pickupBin(Bin* b)
{
    // TODO check that carrier is actually at bin
    if (bin == NULL)
    {
        bin = b;
        state = ShiftingBin;

	//write to file that it has bin
	writeToFile(unique_id,robotType,"Picking up bin");
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
    goals.push_back(binDropOffLocation);
    // Check if carrier has bin
    if (bin != NULL)
    {
        goals.push_back(binDropOffLocation);
	
	//write bin location
	double x = binDropOffLocation.x;
	double y = binDropOffLocation.y;	

	std::string line = "Moving full bin to: ";
	std::string result = line + std::to_string (x) + "," + std::to_string (y);
	writeToFile(unique_id,robotType,result);
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
        ROS_INFO("Carrier reached bin");
	writeToFile(unique_id,robotType,"Carrier reached bin");
        deliverBin();
        goalIndex++;
        state = ShiftingBin;
    }
    else if (state == MovingToInitial)
    {
        ROS_INFO("Carrier reached inital position");
	writeToFile(unique_id,robotType,"Carrier reached inital position");
        // Reached inital position
        state = Idle;
    }
    else if (state == ShiftingBin)
    {
        ROS_INFO("Carrier reached dropoff point");
	writeToFile(unique_id,robotType,"Carrier reached inital position");
        // Reached drop off point.
        dropBin();
        // Check if carrier has other bin goals
        if (binGoals.size() > 0)
        {
            // Bins need picking up do in last come first served order
            ROS_INFO("Carrier still has more bins to pick up");
			writeToFile(unique_id,robotType,"Carrier still has more bins to pick up");
            goals.push_back(binGoals.back());
            binGoals.pop_back();
            state = MovingToBin;
        }
        else
        {
            // No bins need picking up, move back to home
            ROS_INFO("Carrier moving to home");
			writeToFile(unique_id,robotType,"Carrier moving to home");
            if (goals.size() == 0)
            {
                goalIndex = 0;
                goals.push_back(home);
                state = MovingToInitial;
            }
            else
            {
                goalIndex++;
            }
        }
    }

}


void Carrier::reachedLastGoal()
{
    goalIndex = 0;
    reachedCurrentGoal();
    // When carrier has no more goals, it should move back to it's start goal and wait
    // If carrier recieves new goal while it is heading back to home, it should switch
    // to this new goal.

}

