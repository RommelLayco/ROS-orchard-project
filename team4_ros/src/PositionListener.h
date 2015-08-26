#ifndef POS_H
#define POS_H

#include <team4_ros/binIsFull.h>


enum CollisionType {Static, Dynamic};
class Robot; // Necessary due to circular dependencies in #includes

class PositionListener
{
    public:
        PositionListener(std::vector<Robot*> entities);
        CollisionType getCollisionType(geometry_msgs::Point objectLocation, Robot* entity);
    private:
        // Ros handle and subscriber
        ros::NodeHandle subscriberHandle;
        ros::Subscriber binFullSub;

        std::vector<Robot*> entityList;

        void binFullCallback(const team4_ros::binIsFull::ConstPtr& msg);
        
};

#endif
