#ifndef POS_H
#define POS_H

enum CollisionType {Static, Dynamic};
class Robot;

class PositionListener
{
    public:
        PositionListener(std::vector<Robot*> entities);
        CollisionType getCollisionType(geometry_msgs::Point objectLocation);
    private:
        std::vector<Robot*> entityList;
        
};

#endif
