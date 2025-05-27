#include "joystick.h"
#include <Arduino.h>

Joystick::Joystick(int switch_pin,
                   int x_pin,
                   int y_pin) : switch_pin_(switch_pin),
                                x_pin_(x_pin),
                                y_pin_(y_pin)
{
    x_deadband_ = 100;
    y_deadband_ = 100;
}

void Joystick::init(int x_offset, int y_offset)
{
    pinMode(switch_pin_, INPUT_PULLUP);
    pinMode(x_pin_, INPUT);
    pinMode(y_pin_, INPUT);
    x_deadband_ = x_offset;
    y_deadband_ = y_offset;
}

// Return a value between -1 and 1, where 0 is the middle
float Joystick::get_x()
{
    return get_analog_percentage(x_pin_, x_deadband_);
}

float Joystick::get_y()
{
    return get_analog_percentage(y_pin_, y_deadband_);
}

float Joystick::get_analog_percentage(const int pin, int deadband = 0)
{
    int raw = analogRead(pin); // 0…4095
    const int mid = 2048;
    int delta = raw - mid; // –2048…+2047

    // inside dead-band?
    if (abs(delta) <= deadband)
    {
        return 0.0f;
    }

    // remaining span on each side
    int maxRange = mid - deadband; // e.g. if deadband=200 → 1848

    // note: abs(delta) > deadband here
    float scaled = float(abs(delta) - deadband) / float(maxRange);

    // restore sign
    float out = (delta > 0) ? scaled
                            : -scaled;

    // just in case of overshoot
    if (out > 1.0f)
        return 1.0f;
    if (out < -1.0f)
        return -1.0f;
    return out;
}

bool Joystick::button_pressed()
{
    return digitalRead(switch_pin_) == LOW;
}

float Joystick::get_raw_x()
{
    return analogRead(x_pin_);
}

float Joystick::get_raw_y()
{
    return analogRead(y_pin_);
}