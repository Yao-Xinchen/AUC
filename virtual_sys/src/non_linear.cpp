#include "virtual_sys/non_linear.h"

NonLinear::NonLinear(function<Vector2d(Vector2d, double)> f,
    function<double(Vector2d, double)> g,
    double time_ratio)
    : VirtualSys(time_ratio)
{
    this->f_ = f;
    this->g_ = g;
}

Vector2d NonLinear::f(Vector2d x, double u)
{
    return f_(x, u);
}

double NonLinear::g(Vector2d x, double u)
{
    return g_(x, u);
}