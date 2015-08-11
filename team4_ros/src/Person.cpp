#include "Robot.h"


class Person: public Robot
{
    public:
        Person(double x, double y, double z) : Robot(x, y, z) {}
    protected:
        virtual void leftCollisionDetected();
};


void Person::leftCollisionDetected()
{
    ROS_INFO("SUBCLASS!");
}
