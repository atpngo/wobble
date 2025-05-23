#include "joystick.h"
#include <Arduino.h>

Joystick::Joystick(int switch_pin,
                   int x_pin,
                   int y_pin) : switch_pin_(switch_pin),
                                x_pin_(x_pin),
                                y_pin_(y_pin)
{
}

void Joystick::init()
{
    pinMode(switch_pin_, INPUT_PULLUP);
}

// Return a value between -1 and 1, where 0 is the middle
float Joystick::get_x()
{
    return get_analog_percentage(x_pin_);
}

float Joystick::get_y()
{
    return get_analog_percentage(y_pin_);
}

float Joystick::get_analog_percentage(const int pin)
{
    return float(analogRead(pin) - 2048) / 2048.0;
}

bool Joystick::button_pressed()
{
    return digitalRead(switch_pin_) == LOW;
}