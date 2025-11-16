#include "place_nav/place_pose_server.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

PlacePoseServer::PlacePoseServer() : Node("place_pose_server") {
    action_get_place_pose = rclcpp_action::create_server<GetPlacePose>
    (
        this,
        "get_place_pose",
        [](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GetPlacePose::Goal> goal) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetPlacePose>> goal_handle) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetPlacePose>> goal_handle) {
            std::thread([this, goal_handle] {
                this->calculate_place_pose(goal_handle);
            }).detach();
        }
    );
    m_place_section_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        "place_section",
        10
    );
    timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PlacePoseServer::publish_place_section, this));

    m_global_costmap_cli = this->create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");

    publish_place_section();
}

PlacePoseServer::~PlacePoseServer() {

}

void PlacePoseServer::calculate_place_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetPlacePose>> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calculate_place_pose");

    if (!m_global_costmap_cli->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(this->get_logger(), "Service /global_costmap/get_costmap not available!");
        return;
    }

    auto costmap_request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    auto future = m_global_costmap_cli->async_send_request(costmap_request);
    RCLCPP_INFO(this->get_logger(), "Service Call");

    auto costmap_response = future.get();
    const auto &map = costmap_response->map;
    int width = map.metadata.size_x;
    int height = map.metadata.size_y;
    double resolution = map.metadata.resolution;

    RCLCPP_INFO(this->get_logger(), "Got costmap: %dx%d (resolution=%.2f)", width, height, resolution);

    const auto goal = goal_handle->get_goal();

    auto place_poly = place_polygon[goal->place];
    auto x_sum = 0.0;
    auto y_sum = 0.0;
    for (auto& p : place_poly) {
        x_sum += p.first;
        y_sum += p.second;
    }
    
    auto result = std::make_shared<GetPlacePose::Result>();

    if (rclcpp::ok()) {
        goal_handle->succeed(result);
    }

    // response->pose.x = x_sum / place_poly.size();;
    // response->pose.y = y_sum / place_poly.size();
}

void PlacePoseServer::publish_place_section() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    marker.lifetime = rclcpp::Duration(0, 0);

    int i = 0;
    geometry_msgs::msg::Point pt;
    pt.z = 0.0;
    for (auto& poly : place_polygon) {
        marker.header.stamp = this->now();
        marker.id = i++;
        for (auto& p : poly.second) {
            
            pt.x = p.first;
            pt.y = p.second;
            marker.points.push_back(pt);
        }
        pt.x = poly.second[0].first;
        pt.y = poly.second[0].second;
        marker.points.push_back(pt);
        m_place_section_pub->publish(marker);
        marker.points.clear();
    }
    
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<PlacePoseServer>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), // 기본 옵션
        4                          // 쓰레드 수
    );
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}