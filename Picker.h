#ifndef PICKER_H
#define PICKER_H


class Picker
{
    public:
        Picker();

        double getXspeed();
        double getYSpeed();
        double getAngularSpeed();


    protected:
    private:
        double pickerSpeed_X;
        double pickerSpeed_Y;
        double angular_V;
};

#endif // PICKER_H
