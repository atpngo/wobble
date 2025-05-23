#ifndef JOYSTICK_H
#define JOYSTICK_H

class Joystick
{
public:
    Joystick(const int switch_pin, const int x_pin, const int y_pin);
    float get_x();
    float get_y();
    bool button_pressed();

private:
    float get_analog_percentage(const int pin);
    const int switch_pin_;
    const int x_pin_;
    const int y_pin_;
};

#endif // JOYSTICK_H