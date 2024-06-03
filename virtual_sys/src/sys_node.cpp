#include "rclcpp/rclcpp.hpp"
#include "virtual_sys/linear.h"

#include "std_msgs/msg/float64.hpp"
#include <memory>

#define PUB_RATE 50 // ms

class SysNode : public rclcpp::Node
{
public:
    SysNode() : Node("sys_node")
    {
        // a mass-spring-damper system
        double a11 = 0, a12 = 1,
            a21 = -1, a22 = -0;
        double b1 = 0, b2 = 1;
        double c1 = 1, c2 = 0;
        double d = 0;
        double time_ratio = this->declare_parameter("time_ratio", 1.0);
        sys = std::make_unique<Linear>(a11, a12, a21, a22, b1, b2, c1, c2, d, time_ratio);

        // rclcpp
        pub = this->create_publisher<std_msgs::msg::Float64>("output", 10);
        timer = this->create_wall_timer(
            std::chrono::milliseconds(PUB_RATE),
            std::bind(&SysNode::timer_callback, this));

        // start the system
        sys->start(10, 0);

        RCLCPP_INFO(this->get_logger(), "SysNode initialized");
    }

    ~SysNode()
    {
        sys->stop();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<Linear> sys;

    void timer_callback()
    {
        if (!sys->is_running()) return;
        auto msg = std_msgs::msg::Float64();
        msg.data = sys->output();
        pub->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SysNode>());
    rclcpp::shutdown();
    return 0;
}