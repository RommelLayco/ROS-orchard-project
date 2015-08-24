#include "PositionListener.h"

PositionListener::PositionListener(std::vector<Robot*> entities)
{
    entityList = entities;
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
