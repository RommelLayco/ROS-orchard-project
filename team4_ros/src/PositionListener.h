#ifndef POS_H
#define POS_H

enum CollisionType {Static, Dynamic};
class Robot; // Necessary due to circular dependencies in #includes

class PositionListener
{
    public:
        PositionListener(std::vector<Robot*> entities);
        CollisionType getCollisionType(geometry_msgs::Point objectLocation, Robot* entity);
    private:
        std::vector<Robot*> entityList;
        
};

#endif
