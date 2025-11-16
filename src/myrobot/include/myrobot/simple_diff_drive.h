#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class SimpleDiffDrive : public rclcpp::Node
{
private:
    const double WheelRadius = 0.04;
    const double WheelSeparation = 0.17;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::TimerBase::SharedPtr m_timer;
    double m_left_vel, m_right_vel;
    double m_x, m_y, m_theta;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void update();

public : SimpleDiffDrive();
};