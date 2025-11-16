#include "place_nav/place_navigation.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <thread>
#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

PlaceNavigation::PlaceNavigation() : rclcpp::Node("place_navgiation") {
    initialize();
    m_robot_pose_sub = this->create_subscription<PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&PlaceNavigation::robot_pose_callback, this, std::placeholders::_1)
    );

    m_goal_pose_pub = this->create_publisher<PoseStamped>(
        "/goal_pose",
        10
    );

    m_place_section_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        "place_section",
        10
    );

    timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PlaceNavigation::publish_place_section, this));

    m_global_costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap",
        10,
        std::bind(&PlaceNavigation::global_costmap_callback, this, std::placeholders::_1));

    action_goal_pose_server = rclcpp_action::create_server<ActionComputePlacePose>(
        this,
        "compute_place_pose",
        std::bind(&PlaceNavigation::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlaceNavigation::handle_cancel, this, std::placeholders::_1),
        std::bind(&PlaceNavigation::handle_accepted, this, std::placeholders::_1));

    action_goal_pose_client = rclcpp_action::create_client<ActionComputePlacePose>(this, "/compute_place_pose");

    goal_place_sub = this->create_subscription<std_msgs::msg::String>(
        "/goal_place",
        10,
        std::bind(&PlaceNavigation::goal_place_callback, this, std::placeholders::_1)
    );

        std::thread(&PlaceNavigation::display_costmap, this)
            .detach();
}

PlaceNavigation::~PlaceNavigation() {
    
}

void PlaceNavigation::initialize() {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("place_nav");
    std::string yaml_file = pkg_path + "/config/places.yaml";
    YAML::Node places = YAML::LoadFile(yaml_file);
    for (auto place : places)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s : ", place.first.as<std::string>().c_str());

        for (auto point : place.second["polygon"])
        {
            double x = point[0].as<double>();
            double y = point[1].as<double>();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x : %f, y : %f", x, y);
            place_polygon[place.first.as<std::string>()].push_back(std::make_pair(x, y));
        }
    }
}

void PlaceNavigation::robot_pose_callback(PoseWithCovarianceStamped::UniquePtr msg) {
    auto position = msg->pose.pose.position;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot pose :\nx : %f, y: %f, z : %f", position.x, position.y, position.z);

    m_robot_pose = *msg;
}

void PlaceNavigation::global_costmap_callback(OccupancyGrid::UniquePtr msg) {
    m_global_costmap = *msg;
}

void PlaceNavigation::goal_place_callback(std_msgs::msg::String::UniquePtr msg) {
    if (msg->data != "")
        place = msg->data;

    if (!action_goal_pose_client->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(this->get_logger(), "ComputePlacePose action server not available");
        return;
    }

    auto send_goal_options = rclcpp_action::Client<ActionComputePlacePose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionComputePlacePose>> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
    };

    send_goal_options.result_callback =
        [this, &msg](const rclcpp_action::ClientGoalHandle<ActionComputePlacePose>::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Result received: x=%f, y=%f", result.result->goal.pose.position.x, result.result->goal.pose.position.y);
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose = result.result->goal.pose;

            m_goal_pose_pub->publish(pose_msg);
            RCLCPP_INFO(this->get_logger(), "Published goal_pose from place_id: %s", msg->data.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Goal failed");
        }
    };

    auto goal_msg = ActionComputePlacePose::Goal();
    goal_msg.place = place;

    action_goal_pose_client->async_send_goal(goal_msg, send_goal_options);
}

void PlaceNavigation::publish_place_section()
{
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
    for (auto &poly : place_polygon)
    {
        marker.header.stamp = this->now();
        marker.id = i++;
        for (auto &p : poly.second)
        {
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

geometry_msgs::msg::Quaternion PlaceNavigation::calculate_orientation(geometry_msgs::msg::Point start,
                                                                      geometry_msgs::msg::Point end)
{
    auto dx = end.x - start.x;
    auto dy = end.y - start.y;
    auto yaw = atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion result;
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    result.w = q.w();

    return result;
}

rclcpp_action::GoalResponse PlaceNavigation::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ActionComputePlacePose::Goal> goal) {
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlaceNavigation::handle_cancel(const std::shared_ptr < rclcpp_action::ServerGoalHandle<ActionComputePlacePose>> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlaceNavigation::handle_accepted(const std::shared_ptr < rclcpp_action::ServerGoalHandle<ActionComputePlacePose>> goal_handle) {
    auto execute_in_thread = [this, goal_handle]() { return this->execute(goal_handle); };
    std::thread{execute_in_thread}.detach();
}

void PlaceNavigation::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionComputePlacePose>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto goal = goal_handle->get_goal();
    auto action_result = std::make_shared<ActionComputePlacePose::Result>();
    int width = m_global_costmap.info.width;
    int height = m_global_costmap.info.height;
    if (width <= 0 || height <= 0) {
        goal_handle->abort(action_result);
    }
    const std::vector<int8_t> &data = m_global_costmap.data;

    RCLCPP_INFO(this->get_logger(), "Costmap Callback Map width: %d, height: %d", width, height);

    cv::Mat map_img(height, width, CV_8UC1);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int8_t value = data[y * width + x];
            uint8_t pixel;

            if (value > 80)
                pixel = 0;
            else
                pixel = 255;

            map_img.at<uchar>(y, x) = pixel;
        }
    }

    cv::resize(map_img, map_img, cv::Size(width * 5, height * 5));

    cv::Mat mask = cv::Mat::zeros(height * 5, width * 5, CV_8UC1);
    std::vector<cv::Point> poly;

    auto goal_place = goal->place;
    if (goal_place == "") {
        goal_place = this->place;
    }
    for (auto& p : place_polygon[goal_place]) { 
        poly.push_back(convert_meter_to_pixel_pose(p));
    }
    std::vector<std::vector<cv::Point>> polys = {poly};
    cv::fillPoly(mask, polys, cv::Scalar(255));
    cv::bitwise_and(map_img, mask, map_img);

    cv::Mat dist;
    distanceTransform(map_img, dist, cv::DIST_L2, 3);
    normalize(dist, dist, 0, 1.0, cv::NORM_MINMAX);

    double max_val;
    cv::Point max_loc;
    cv::minMaxLoc(dist, nullptr, &max_val, nullptr, &max_loc);
    cv::Mat color_img;
    cv::cvtColor(map_img, color_img, cv::COLOR_GRAY2BGR);
    cv::circle(color_img, max_loc, 2, cv::Scalar(0, 0, 255), 2);
    cv::resize(color_img, color_img, cv::Size(width, height));
    m_image_queue.push(color_img);

    auto goal_pose = convert_pixel_to_meter_pose(max_loc);

    action_result->goal.header.frame_id = "map";
    action_result->goal.header.stamp = this->now();
    action_result->goal.pose.position.x = goal_pose.first;
    action_result->goal.pose.position.y = goal_pose.second;
    action_result->goal.pose.orientation = calculate_orientation(m_robot_pose.pose.pose.position, action_result->goal.pose.position);

    RCLCPP_INFO(this->get_logger(), "Goal Pose x : %f, y : %f", goal_pose.first, goal_pose.second);
    if (rclcpp::ok()) {
        goal_handle->succeed(action_result);
        //m_goal_pose_pub->publish(action_result->goal);
    }
}

void PlaceNavigation::display_costmap() {
    while (true) {
        cv::Mat img;
        {
            if (!m_image_queue.empty())
            {
                img = m_image_queue.front();
                m_image_queue.pop();
            }
        }

        if (!img.empty())
        {
            std::string win_name = "Occupancy Grid Map ";
            cv::imshow(win_name, img);
            cv::waitKey(1);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

std::pair<double, double> PlaceNavigation::convert_pixel_to_meter_pose(const cv::Point pixel_pose) const {
    if (m_global_costmap.info.width <= 0 || m_global_costmap.info.height <= 0) {
        return std::make_pair(0, 0);
    }
    auto x = pixel_pose.x / 100 + m_global_costmap.info.origin.position.x;
    auto y = pixel_pose.y / 100 + m_global_costmap.info.origin.position.y;
    return std::make_pair(x, y);
}

cv::Point PlaceNavigation::convert_meter_to_pixel_pose(const std::pair<double, double> meter_pose) const {
    if (m_global_costmap.info.width <= 0 || m_global_costmap.info.height <= 0) {
        return cv::Point(0, 0);
    }
    auto x = static_cast<int>((meter_pose.first - m_global_costmap.info.origin.position.x) * 100);
    auto y = static_cast<int>((meter_pose.second - m_global_costmap.info.origin.position.y) * 100);
    return cv::Point(x, y);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<PlaceNavigation>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();

    return 0;
}