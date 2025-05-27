#ifndef JOYSTICK_H
#define JOYSTICK_H

class Joystick
{
public:
    Joystick(int switch_pin, int x_pin, int y_pin);
    void init(int x_deadband, int y_deadband);
    float get_x();
    float get_y();
    bool button_pressed();

    float get_raw_x();
    float get_raw_y();

private:
    float get_analog_percentage(const int pin, int deadband);
    const int switch_pin_;
    const int x_pin_;
    const int y_pin_;
    int x_deadband_;
    int y_deadband_;
};

#endif // JOYSTICK_H