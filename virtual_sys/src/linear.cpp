#include "virtual_sys/linear.h"

Linear::Linear(double a11, double a12, double a21, double a22,
    double b1, double b2, double c1, double c2, double d,
    double time_ratio)
    : VirtualSys(time_ratio)
{
    A << a11, a12, a21, a22;
    B << b1, b2;
    C << c1, c2;
    D = d;
}

Eigen::Vector2d Linear::f(Eigen::Vector2d x, double u)
{
    return A * x + B * u;
}

double Linear::g(Eigen::Vector2d x, double u)
{
    return C * x + D * u;
}