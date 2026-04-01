#ifndef MAP_CREATOR_HPP_
#define MAP_CREATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <opencv2/opencv.hpp>

enum CellValue {
    FREE = 0,
    UNKNOWN = -1,
    OCCUPIED = 100
};

class MapCreator : public rclcpp::Node {
private:
    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

    rclcpp::Subscription<OccupancyGrid>::SharedPtr m_map_sub;
    rclcpp::Publisher<PoseStamped>::SharedPtr m_goal_pub;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr m_pose_sub;
    OccupancyGrid m_map;
    bool m_is_get_robot_pose;
    int robot_map_x, robot_map_y;

    void map_callback(const OccupancyGrid::SharedPtr msg);
    void pose_callback(const PoseWithCovarianceStamped::SharedPtr msg);

    double dist(auto& a, auto& b) const;
    int xy_to_idx(int x, int y) const;
    bool is_valid(int x, int y) const;
    std::vector<std::pair<int, int>> neighbors(int x, int y) const;

    bool is_frontier_cell(int x, int y) const;
    std::vector<std::pair<int, int>> detect_frontiers() const;
    std::vector<std::vector<std::pair<int, int>>> cluster_frontiers(const std::vector<std::pair<int, int>> &frontiers) const;
    geometry_msgs::msg::PoseStamped choose_best_cluster(const std::vector<std::vector<std::pair<int, int>>> &clusters) const;

public:
    MapCreator();
    ~MapCreator();
};
#endif