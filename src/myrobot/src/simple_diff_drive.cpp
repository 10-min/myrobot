#include "simple_diff_drive.h"
#include <memory>
SimpleDiffDrive::SimpleDiffDrive() : Node("simple_diff_drive"), m_left_vel(0), m_right_vel(0),
                                    m_x(0), m_y(0), m_theta(0)
{
    m_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&SimpleDiffDrive::jointStateCallback, this, std::placeholders::_1));
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SimpleDiffDrive::update, this));
}

void SimpleDiffDrive::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "left_wheel_joint")
            m_left_vel = msg->velocity[i];
        if (msg->name[i] == "right_wheel_joint")
            m_right_vel = msg->velocity[i];
    }

    RCLCPP_INFO(this->get_logger(), "Left vel : %f, Right vel : %f", m_left_vel, m_right_vel);
}

void SimpleDiffDrive::update() {
    double dt = 0.02; // 50Hz
    double v = WheelRadius * (m_left_vel + m_right_vel) / 2.0;
    double w = WheelRadius * (m_right_vel - m_left_vel) / WheelSeparation;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleDiffDrive>());
    rclcpp::shutdown();
    return 0;
}