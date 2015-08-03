#include "RobotSuperClass.h"
#include <iostream>

using namespace std;

RobotSuperClass::RobotSuperClass()
{
    cout << "superclass1" << endl;
}

void RobotSuperClass::changeSpeed(double linear_v_x, double linear_v_y,double angular_v){

    linear_velocity_x = linear_v_x;
    linear_velocity_y = linear_v_y;
    angular_velocty = angular_v;

}

void RobotSuperClass::testChangeSpeed(){

    cout << linear_velocity_x << endl;
    cout << linear_velocity_y << endl;
    cout << angular_velocty << endl;

}
