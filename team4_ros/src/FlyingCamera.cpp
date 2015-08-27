#include "Robot.h"
#include <time.h>


class FlyingCamera: public Person
{
public:
    FlyingCamera(int sensor_range, int sensor_angle, int number, std::string type) : Person(sensor_range, sensor_angle,number,type) {}
protected:
};




