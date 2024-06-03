#include "rclcpp/rclcpp.hpp"
#include "pid_controller/pid_algorithm.h"

#include "std_msgs/msg/float64.hpp"
#include <rclcpp/timer.hpp>

class PIDController : public rclcpp::Node
{
public:
    PIDController() : Node("pid_controller")
    {
        double p2v_kp = this->declare_parameter("p2v_kp", 2.0);
        double p2v_ki = this->declare_parameter("p2v_ki", 0.0);
        double p2v_kd = this->declare_parameter("p2v_kd", 1.0);

        double v2a_kp = this->declare_parameter("v2a_kp", 2.0);
        double v2a_ki = this->declare_parameter("v2a_ki", 1.0);
        double v2a_kd = this->declare_parameter("v2a_kd", 0.0);

        p2v_pid = std::make_unique<PIDAlgorithm>(p2v_kp, p2v_ki, p2v_kd);
        v2a_pid = std::make_unique<PIDAlgorithm>(v2a_kp, v2a_ki, v2a_kd);

        p2v_pid->set_target(-2);

        pub = this->create_publisher<std_msgs::msg::Float64>("control", 10);
        position_sub = this->create_subscription<std_msgs::msg::Float64>(
            "output", 10,
            std::bind(&PIDController::position_callback, this, std::placeholders::_1));
        velocity_sub = this->create_subscription<std_msgs::msg::Float64>(
            "output_derivative", 10,
            std::bind(&PIDController::velocity_callback, this, std::placeholders::_1));

        control_timer = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&PIDController::control_callback, this));

        vel_timer = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&PIDController::vel_callback, this));

        RCLCPP_INFO(this->get_logger(), "PIDController initialized");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub;

    rclcpp::TimerBase::SharedPtr vel_timer;
    rclcpp::TimerBase::SharedPtr control_timer;

    std::unique_ptr<PIDAlgorithm> p2v_pid;
    std::unique_ptr<PIDAlgorithm> v2a_pid;

    void position_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        p2v_pid->set_feedback(msg->data);
    }

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        v2a_pid->set_feedback(msg->data);
    }

    void control_callback()
    {
        double control = v2a_pid->get_output();
        std_msgs::msg::Float64 control_msg;
        control_msg.data = control;
        pub->publish(control_msg);
    }

    void vel_callback()
    {
        double vel = p2v_pid->get_output();
        v2a_pid->set_target(vel);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}