// pid.cpp
#include "pid.h"

PID::PID(double kp, double ki, double kd, double dt)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0), prev_error_(0), output_(0), dt_(dt), error_(0) {}

void PID::reset()
{
    integral_ = 0;
    prev_error_ = 0;
    output_ = 0;
    error_ = 0;
}

double PID::get_signal(double current_value, double target)
{
    error_ = current_value - target;
    integral_ += error_ * dt_;
    double derivative = (error_ - prev_error_) / dt_;
    output_ = kp_ * error_ + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error_;
    return output_;
}
