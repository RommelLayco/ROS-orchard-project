#include "../include/team4_ros/Picker.h"

Picker::Picker()
{
    //ctor##########
}

//Obtain x (horizontal) pickerSpeed
double Picker::getXspeed(){
    return pickerSpeed_X;
}

//Obtain y pickerSpeed
double Picker::getYspeed(){
    return pickerSpeed_Y;
}

//Obtain x (angular) pickerSpeed
double Picker::getAngularSpeed(){
    return angular_V;
}
