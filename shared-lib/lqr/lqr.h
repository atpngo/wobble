#include <Eigen/Dense>

using Eigen::Matrix;
using Eigen::Vector2f;

class LQR
{
public:
    LQR(float k_theta, float k_theta_dot);
    float compute(float theta_deg, float theta_dot_deg);

private:
    Eigen::RowVector2f K;
};
