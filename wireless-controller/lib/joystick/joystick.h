#ifndef JOYSTICK_H
#define JOYSTICK_H

class Joystick
{
public:
    Joystick(int switch_pin, int x_pin, int y_pin);
    void init();
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