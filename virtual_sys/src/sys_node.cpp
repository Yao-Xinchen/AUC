#include "rclcpp/rclcpp.hpp"
#include "virtual_sys/linear.h"

#include "std_msgs/msg/float64.hpp"
#include <memory>

#define PUB_RATE 5 // ms

class SysNode : public rclcpp::Node
{
public:
    SysNode() : Node("sys_node")
    {
        // a mass-spring-damper system
        double a11 = 0, a12 = 1,
            a21 = -1, a22 = -1;
        double b1 = 0, b2 = 1;
        double c1 = 1, c2 = 0;
        double d = 0;
        double time_ratio = this->declare_parameter("time_ratio", 1.0);
        sys = std::make_unique<Linear>(a11, a12, a21, a22, b1, b2, c1, c2, d, time_ratio);
        
        // rclcpp
        pub = this->create_publisher<std_msgs::msg::Float64>("output", 10);
        pub_derivative = this->create_publisher<std_msgs::msg::Float64>("output_derivative", 10);
        control_sub = this->create_subscription<std_msgs::msg::Float64>(
            "control", 10,
            std::bind(&SysNode::control_callback, this, std::placeholders::_1));
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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_derivative;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_sub;

    std::unique_ptr<Linear> sys;

    void timer_callback()
    {
        if (!sys->is_running()) return;
        auto output_msg = std_msgs::msg::Float64();
        output_msg.data = sys->output();
        pub->publish(output_msg);

        auto output_derivative_msg = std_msgs::msg::Float64();
        output_derivative_msg.data = sys->output_derivative();
        pub_derivative->publish(output_derivative_msg);
    }

    void control_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        sys->input(msg->data);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SysNode>());
    rclcpp::shutdown();
    return 0;
}