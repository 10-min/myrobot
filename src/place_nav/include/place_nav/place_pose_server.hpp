#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "myrobot_interfaces/action/get_place_pose.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include <unordered_map>
#include <string>
#include <vector>

class PlacePoseServer : public rclcpp::Node {
private:
    using GetPlacePose = myrobot_interfaces::action::GetPlacePose;
    rclcpp_action::Server<GetPlacePose>::SharedPtr action_get_place_pose;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_place_section_pub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr m_global_costmap_cli;

    void calculate_place_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetPlacePose>> goal_handle);

    void publish_place_section();

public : 
    PlacePoseServer();
    ~PlacePoseServer();
};