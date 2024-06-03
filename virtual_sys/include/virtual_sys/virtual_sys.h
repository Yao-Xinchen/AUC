#ifndef VIRTUAL_SYS_H
#define VIRTUAL_SYS_H

#include <array>
#include <thread>

#include <eigen3/Eigen/Dense>

#define VIRTUAL_DT 1000 // us, virtual time step

class VirtualSys
{
public:
    VirtualSys(double time_ratio = 1);

    void start(double x1i, double x2i);

    void stop();

    void input(double u);

    double output();

    bool is_running();

protected:
    int real_dt = 1; // us, real time step

    bool running = false;
    std::thread calc_thread;

    void calc();

    double y = 0;
    double u = 0;
    Eigen::Vector2d x;

    // dx/dt = f(x, u)
    virtual Eigen::Vector2d f(Eigen::Vector2d x, double u) = 0;
    
    // y = g(x, u)
    virtual double g(Eigen::Vector2d x, double u) = 0;
};

#endif // VIRTUAL_SYS_H