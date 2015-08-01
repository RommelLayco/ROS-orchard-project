#ifndef ROBOTSUPERCLASS_H
#define ROBOTSUPERCLASS_H


class RobotSuperClass
{
    public:
        RobotSuperClass();
        void transmit();
    protected:
    private:

        // the attributes of the robot
        double linear_velocity_x;
        double linear_velocity_y;
        double angular_velocty;
        double current_x;
        double current_y;
        double current_theta;
        int id;

        // properties of the physical robot
        double length;
        double width;
        double height;
        double top_speed;
        // the state lets us know what speed to give to the robot
        double current_state;

        void changeSpeed(linear_velocity_x, linear_velocity_y, angular_velocity);

};

#endif // ROBOTSUPERCLASS_H
