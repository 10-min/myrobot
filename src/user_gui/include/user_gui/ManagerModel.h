#ifndef MANAGER_MODEL_H
#define MANAGER_MODEL_H

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "myrobot_interfaces/action/clean_room.hpp"
#include <QImage>

class ManagerModel : public rclcpp::Node {
    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
    using ActionCleanRoom = myrobot_interfaces::action::CleanRoom;
private:
    rclcpp::Subscription<OccupancyGrid>::SharedPtr m_map_sub;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr m_robot_pose_sub;
    rclcpp_action::Client<ActionCleanRoom>::SharedPtr action_clean_room_client;

    OccupancyGrid m_map;
    QImage m_map_image;
    std::pair<double, double> m_robot_pose;

    void mapCallback(const OccupancyGrid::SharedPtr msg);
    QImage occupancyGridToImage(const OccupancyGrid::SharedPtr grid);
    void poseCallback(const PoseWithCovarianceStamped::SharedPtr msg);
    void paintRobotPose(QImage& map_image, std::pair<int, int> map_pose);

public:
    ManagerModel();
    ~ManagerModel() {}

    std::function<void(QImage)> map_handler;
    void cleanRoom();
};

#endif