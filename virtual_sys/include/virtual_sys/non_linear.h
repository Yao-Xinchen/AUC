#ifndef NON_LINEAR_H
#define NON_LINEAR_H

#include <functional>
#include <virtual_sys/virtual_sys.h>

using std::function;
using Eigen::Vector2d;

class NonLinear : public VirtualSys
{
public:
    NonLinear(function<Vector2d(Vector2d, double)> f,
        function<double(Vector2d, double)> g,
        double time_ratio = 1);

private:
    Vector2d f(Vector2d x, double u) override;

    double g(Vector2d x, double u) override;

    function<Vector2d(Vector2d, double)> f_;
    function<double(Vector2d, double)> g_;
};

#endif // NON_LINEAR_H