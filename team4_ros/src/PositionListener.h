#ifndef POS_H
#define POS_H

#include <team4_ros/binIsFull.h>


enum CollisionType {Static, Dynamic};
class Robot; // Necessary due to circular dependencies in #includes

class PositionListener
{
public:
    PositionListener(std::vector<Robot*> entities);

    /**
     * This method is invoked by an entity when it detects a collision and wishes to determine
     * whether it is colliding with a static object (e.g a tree) dynamic object (e.g a dog).
     *
     * @param objectLocation  point representing the coordinates of the object the entity is colliding with
     * @param entity		 the entity that is calling this method
     * @return		 the type of collsion (Static or Dynamic)
     */
    CollisionType getCollisionType(geometry_msgs::Point objectLocation, Robot* entity);
private:
    // Ros handle and subscriber
    ros::NodeHandle subscriberHandle;
    ros::Subscriber binFullSub;

    std::vector<Robot*> entityList;

    /**
     * This method is invoked by ROS a picker publishes a bin full message
     *
     * @param msg  the binIsFull message
     */
    void binFullCallback(const team4_ros::binIsFull::ConstPtr& msg);
};

#endif

