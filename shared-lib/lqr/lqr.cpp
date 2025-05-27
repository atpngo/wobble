#include "lqr.h"

LQR::LQR(float k_theta, float k_theta_dot)
{
    // K << 12.5f, 2.3f;
    K << k_theta, k_theta_dot;
}

float LQR::compute(float theta_deg, float theta_dot_deg)
{
    float theta = theta_deg * M_PI / 180.0f;
    float theta_dot = theta_dot_deg * M_PI / 180.0f;

    Vector2f x(theta, theta_dot);
    float u = -(K * x)(0, 0);
    return u;
}