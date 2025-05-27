#ifndef MOTOR_H
#define MOTOR_H

enum MotorType
{
    MAIN,
    SECONDARY
};

enum Direction
{
    FORWARD,
    BACKWARD
};

class Motor
{
public:
    Motor(const int a_pin, const int b_pin, const int enable_pin);
    void set_type(MotorType type);
    void follow(Motor other);
    void spin(int power); // signal between -255 and 255
private:
    const int a_pin_;
    const int b_pin_;
    const int enable_pin_;
    MotorType type_;

    void set_direction(Direction dir);
};

#endif // MOTOR_H