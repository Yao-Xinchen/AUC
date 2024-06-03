#include "virtual_sys/virtual_sys.h"
#include <rclcpp/rclcpp.hpp>

VirtualSys::VirtualSys(double time_ratio)
{
    real_dt = VIRTUAL_DT / time_ratio;
    RCLCPP_INFO(rclcpp::get_logger("VirtualSys"), "time_ratio: %3f", time_ratio);
}

void VirtualSys::start(double x1i, double x2i)
{
    x << x1i, x2i;
    y = g(x, u);
    running = true;
    calc_thread = std::thread(&VirtualSys::calc, this);
    RCLCPP_INFO(rclcpp::get_logger("VirtualSys"), "VirtualSys started");
}

void VirtualSys::stop()
{
    running = false;
    if (calc_thread.joinable()) calc_thread.join();
    RCLCPP_INFO(rclcpp::get_logger("VirtualSys"), "VirtualSys stopped");
}

void VirtualSys::calc()
{
    while (running)
    {
        y = g(x, u);
        constexpr double dt = VIRTUAL_DT / 1e6;
        x += f(x, u) * dt;
        std::this_thread::sleep_for(std::chrono::microseconds(real_dt));
    }
}

void VirtualSys::input(double u)
{
    this->u = u;
}

double VirtualSys::output()
{
    return y;
}

bool VirtualSys::is_running()
{
    return running;
}