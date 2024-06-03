#ifndef LINEAR_H
#define LINEAR_H

#include <virtual_sys/virtual_sys.h>

class Linear : public VirtualSys
{
public:
    Linear(double a11, double a12, double a21, double a22,
        double b1, double b2, double c1, double c2, double d,
        double time_ratio = 1);

private:
    // state space model
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::RowVector2d C;
    double D;

    Eigen::Vector2d f(Eigen::Vector2d x, double u) override;

    double g(Eigen::Vector2d x, double u) override;
};

#endif // LINEAR_H