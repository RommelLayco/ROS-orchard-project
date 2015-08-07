#include <iostream>
#include "../include/team4_ros/RobotSuperClass.h"

using namespace std;
//this is comment
int main()
{
    RobotSuperClass r;

    r.changeSpeed(1.0,2.0,3.0);

    r.testChangeSpeed();

    return 0;
}
