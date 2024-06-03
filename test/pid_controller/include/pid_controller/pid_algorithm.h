#ifndef PID_ALGORITHM_H
#define PID_ALGORITHM_H

#include <thread>

#define VIRTUAL_DT 1000 // us, virtual time step

class PIDAlgorithm
{
public:
    PIDAlgorithm(double kp, double ki, double kd, double time_ratio = 1);

    ~PIDAlgorithm();

    void set_target(double x);

    void set_feedback(double x);

    double get_output();

private:
    double kp, ki, kd;
    int real_dt = 1; // us, real time step

    double feedback = 0;
    double target = 0;
    double output = 0;

    std::thread calc_thread;

    void calc();
};

#endif // PID_ALGORITHM_H