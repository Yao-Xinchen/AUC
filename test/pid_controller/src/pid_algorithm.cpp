#include "pid_controller/pid_algorithm.h"
#include <rclcpp/rclcpp.hpp>

PIDAlgorithm::PIDAlgorithm(double kp, double ki, double kd, double time_ratio)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    real_dt = VIRTUAL_DT / time_ratio;

    calc_thread = std::thread(&PIDAlgorithm::calc, this);
}

PIDAlgorithm::~PIDAlgorithm()
{
    if (calc_thread.joinable()) calc_thread.join();
}

void PIDAlgorithm::set_target(double x)
{
    target = x;
}

void PIDAlgorithm::set_feedback(double x)
{
    feedback = x;
}

double PIDAlgorithm::get_output()
{
    return output;
}

void PIDAlgorithm::calc()
{
    double error = 0;
    double integral = 0;
    double derivative = 0;
    double prev_error = 0;

    double dt = VIRTUAL_DT / 1e6;

    while (rclcpp::ok())
    {
        error = target - feedback;
        integral += error * dt;
        derivative = (error - prev_error) / dt;
        prev_error = error;

        output = kp * error + ki * integral + kd * derivative;

        std::this_thread::sleep_for(std::chrono::microseconds(real_dt));
    }
}