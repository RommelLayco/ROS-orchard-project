#include "PositionListener.h"
#include <math.h>
#include "Util.cpp"

PositionListener::PositionListener(std::vector<Robot*> entities)
{
    entityList = entities;
	// Subscribe to bin topic to find out when a picker's bin is full
    binFullSub = subscriberHandle.subscribe("bin_topic", 1000, &PositionListener::binFullCallback, this);
}

CollisionType PositionListener::getCollisionType(geometry_msgs::Point objectLocation, Robot* entity)
{
    double tolerance = 1.5;

    for (int i = 0; i < entityList.size(); i++)
    {
        double x = entityList[i]->getXPos();
        double y = entityList[i]->getYPos();
        
        if (fabs(x - objectLocation.x) <= tolerance && fabs(y - objectLocation.y) <= tolerance && entityList[i] != entity)
        {
            // There is a dynamic entity near where the collision happened
            return Dynamic;
        }
    }
    
    return Static;
}

void PositionListener::binFullCallback(const team4_ros::binIsFull::ConstPtr& msg)
{
    std::vector<Robot*> carriers;
    // Determine which carrier is closest to the bin. Only choose a carrier that is not already carrying a bin.
    ROS_INFO("Recieved bin is full message!");
    for (int i = 0; i < entityList.size(); i++)
    {
        std::string entityType = entityList[i]->robotType;
        std::string carrier = "carrier";
        if (entityType.find(carrier) != std::string::npos)
        {
            carriers.push_back(entityList[i]);
        }
    }

    double binX = msg->x;
    double binY = msg->y;
    double shortestDist = 0;
    int index = 0;

    for (int i = 0; i < carriers.size(); i++)
    {
        if (Util::getDistance(binX, binY, carriers[i]->getXPos(), carriers[i]->getYPos()) > shortestDist)
        {
            index = i;
        }
    }

    if (carriers.size() > 0 )
    {
        geometry_msgs::Point point;
        point.x = binX;
        point.y = binY;
        Carrier* carrier = dynamic_cast<Carrier*>(carriers[index]);
        // TODO handle case where closest carrier already has a bin
        carrier->moveToBin(point);
        ROS_INFO("Sent carrier to pick up bin");
    }

}

