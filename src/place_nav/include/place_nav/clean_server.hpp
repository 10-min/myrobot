#ifndef CLEAN_SERVER_HPP_
#define CLEAN_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "myrobot_interfaces/action/clean_room.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <list>
#include <queue>

struct Rect {
    int x, y, width, height, area;
};

struct DistCompare {
    bool operator() (const std::pair<double, std::pair<int, int>>& a, const std::pair<double, std::pair<int, int>>& b) {
        return a.first > b.first;
    }
};

enum RobotState {
    IDLE,
    MOVING_TO_START,
    CLEANING_LINE,
    MOVING_TO_NEXT_LINE
};

class CleanServer : public rclcpp::Node {
    using FollowPath = nav2_msgs::action::FollowPath;
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    
    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using ActionCleanRoom = myrobot_interfaces::action::CleanRoom;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

private:
    rclcpp::Subscription<OccupancyGrid>::SharedPtr m_map_sub;
    rclcpp_action::Client<NavigateToPose>::SharedPtr m_navigate_client;
    rclcpp_action::Client<FollowPath>::SharedPtr m_follow_client;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp_action::Server<ActionCleanRoom>::SharedPtr action_clean_room_server;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr m_pose_sub;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr m_costmap_sub;
    rclcpp::TimerBase::SharedPtr m_clean_timer;

    OccupancyGrid m_cleaning_space;
    OccupancyGrid m_costmap;
    std::list<Rect> m_cleaning_rect;
    std::pair<int, int> m_robot_grid_pose;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionCleanRoom>> clean_goal_handle;
    rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr follow_goal_handle;
    Rect *current_rect = nullptr;
    bool is_rect_cleaned;
    bool left_to_right = true;
    int current_y = 0;
    RobotState m_robot_state = RobotState::IDLE;
    bool is_end_of_line = false;
    int y_step;

    void map_callback(const OccupancyGrid::SharedPtr msg);
    void costmap_callback(const OccupancyGrid::SharedPtr msg);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ActionCleanRoom::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionCleanRoom>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionCleanRoom>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionCleanRoom>> goal_handle);
    void pose_callback(const PoseWithCovarianceStamped::SharedPtr msg);
    void cleanRect();
    void onNavResult(const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult &result);
    std::vector<std::pair<int, int>> astar(std::pair<int, int> start, std::pair<int, int> goal, const OccupancyGrid &map);
    nav_msgs::msg::Path makePathFromWaypoints(const std::vector<std::pair<int,int>>& waypoints) const;
    void sendFollowPath(const nav_msgs::msg::Path& path);
    void checkCleanDone();

public : 
    CleanServer();
    ~CleanServer();
};

#endif