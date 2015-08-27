#include <geometry_msgs/Twist.h>


class SpeedListener
{
public:
    virtual void speedUpdate(geometry_msgs::Twist speedMsg) = 0;
};

