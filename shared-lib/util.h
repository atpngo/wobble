#ifndef UTIL_H
#define UTIL_H
#include <cmath>

inline double approximate_to_zero(double value, double epsilon = 1e-6)
{
    return (std::abs(value) < epsilon) ? 0.0 : value;
}

#endif // UTIL_H
