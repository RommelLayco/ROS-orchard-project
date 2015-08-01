#include <iostream>
#include "RobotSuperClass.h"

using namespace std;

int main()
{
    RobotSuperClass r;

    r.changeSpeed(1.0,2.0,3.0);

    r.testChangeSpeed();

    return 0;
}
