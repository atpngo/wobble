#include "motor.h"
#include <Arduino.h>
#include <cmath>

Motor::Motor(const int a_pin,
             const int b_pin,
             const int enable_pin) : a_pin_(a_pin),
                                     b_pin_(b_pin),
                                     enable_pin_(enable_pin)
{
    pinMode(a_pin_, OUTPUT);
    pinMode(b_pin_, OUTPUT);
    pinMode(enable_pin_, OUTPUT);
    type_ = MotorType::MAIN;
}

void Motor::set_type(MotorType type)
{
    type_ = type;
}

void Motor::follow(Motor other)
{
}

void Motor::spin(int power)
{
    if (power > 0)
    {
        set_direction(Direction::FORWARD);
    }
    else
    {
        set_direction(Direction::BACKWARD);
    }
    analogWrite(enable_pin_, abs(power));
}

void Motor::set_direction(Direction dir)
{
    if ((dir == Direction::FORWARD && type_ == MotorType::MAIN) || (dir == Direction::BACKWARD && type_ == MotorType::SECONDARY))
    {
        digitalWrite(a_pin_, LOW);
        digitalWrite(b_pin_, HIGH);
    }
    else
    {
        digitalWrite(a_pin_, HIGH);
        digitalWrite(b_pin_, LOW);
    }
}
