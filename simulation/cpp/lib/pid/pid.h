// pid.h
#pragma once

class PID
{
public:
    PID(double kp, double ki, double kd, double dt);

    void reset();
    double get_signal(double current_value, double target);

private:
    double kp_, ki_, kd_;
    double integral_, prev_error_, output_;
    double dt_;
    double error_;
};
