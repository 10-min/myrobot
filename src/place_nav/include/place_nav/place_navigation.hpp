#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "myrobot_interfaces/action/compute_place_pose.hpp"
#include "myrobot_interfaces/action/get_place_pose.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include <queue>
#include <opencv2/opencv.hpp>

class PlaceNavigation : public rclcpp::Node {
private:
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using ActionComputePlacePose = myrobot_interfaces::action::ComputePlacePose;
    using GetPlacePose = myrobot_interfaces::action::GetPlacePose;
    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr m_robot_pose_sub;
    rclcpp::Publisher<PoseStamped>::SharedPtr m_goal_pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_place_section_pub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_global_costmap_sub;
    rclcpp_action::Server<ActionComputePlacePose>::SharedPtr action_goal_pose_server;
    rclcpp_action::Client<ActionComputePlacePose>::SharedPtr action_goal_pose_client;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_place_sub;

    PoseWithCovarianceStamped m_robot_pose;
    std::unordered_map<std::string, std::vector<std::pair<double, double>>> place_polygon;
    OccupancyGrid m_global_costmap;
    std::queue<cv::Mat> m_image_queue;
    std::string place;

    void initialize();

    void robot_pose_callback(PoseWithCovarianceStamped::UniquePtr msg);
    geometry_msgs::msg::Quaternion calculate_orientation(geometry_msgs::msg::Point start,
                                                         geometry_msgs::msg::Point end);

    void global_costmap_callback(OccupancyGrid::UniquePtr msg);
    
    void goal_place_callback(std_msgs::msg::String::UniquePtr msg);

    void publish_place_section();

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ActionComputePlacePose::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionComputePlacePose>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionComputePlacePose>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionComputePlacePose>> goal_handle);    
    void display_costmap();
    std::pair<double, double> convert_pixel_to_meter_pose(const cv::Point pixel_pose) const;
    cv::Point convert_meter_to_pixel_pose(const std::pair<double, double> meter_pose) const;

public:
    PlaceNavigation();
    ~PlaceNavigation();
};