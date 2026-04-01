#include "place_nav/clean_server.hpp"
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

CleanServer::CleanServer(): Node("clean_server") {
    using namespace std::chrono_literals;

    m_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&CleanServer::map_callback, this, std::placeholders::_1));

    m_navigate_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    m_follow_client = rclcpp_action::create_client<FollowPath>(this, "follow_path");

    m_path_pub = this->create_publisher<nav_msgs::msg::Path>("/plan", 1);

    action_clean_room_server = rclcpp_action::create_server<ActionCleanRoom>(
        this,
        "clean_room",
        std::bind(&CleanServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CleanServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&CleanServer::handle_accepted, this, std::placeholders::_1));

    m_pose_sub = this->create_subscription<PoseWithCovarianceStamped>(
        "/amcl_pose",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&CleanServer::pose_callback, this, std::placeholders::_1));

    m_costmap_sub = this->create_subscription<OccupancyGrid>(
        "global_costmap/costmap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&CleanServer::costmap_callback, this, std::placeholders::_1));

    m_clean_timer = this->create_wall_timer(1s, std::bind(&CleanServer::cleanRect, this));
    m_clean_timer->cancel();
}

CleanServer::~CleanServer() {

}

void CleanServer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    
    int width = msg->info.width;
    int height = msg->info.height;

    if (width <= 0 || height <= 0) return;

    y_step  = 0.22 / msg->info.resolution;
    m_cleaning_space = *msg.get();

    RCLCPP_INFO(this->get_logger(), "Map Callback Map width: %d, height: %d", width, height);

    cv::Mat map_img(height, width, CV_8UC1);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++)
        {
            int8_t value = m_cleaning_space.data[y * width + x];
            uint8_t pixel;

            if (value >= 100)
                pixel = 0;
            else
                pixel = 255;

            map_img.at<uchar>(y, x) = pixel;
        }
    }

    cv::Mat filled = map_img.clone();
    cv::Mat mask(filled.rows + 2, filled.cols + 2, CV_8UC1, cv::Scalar(0));
    cv::floodFill(filled, mask, cv::Point(0, 0), 128);

    for (int y = 0; y < filled.rows; ++y) {
        for (int x = 0; x < filled.cols; ++x) {
            if (filled.at<uchar>(y, x) == 128) {
                m_cleaning_space.data[y * width + x] = 100;
            } else {
                if (m_cleaning_space.data[y * width + x] != 100) {
                    m_cleaning_space.data[y * width + x] = 0;
                }
            }
        }
    }

    cv::Mat cleaning_space_img(height, width, CV_8UC1);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int8_t value = m_cleaning_space.data[y * width + x];
            uint8_t pixel;

            if (value == 100)
                pixel = 0;
            else
                pixel = 255;

            cleaning_space_img.at<uchar>(y, x) = pixel;
        }
    }

    cv::Mat rectangle_covered_map = cleaning_space_img.clone();
    cv::Mat color_rectangle_covered_map = cleaning_space_img.clone();
    cv::cvtColor(rectangle_covered_map, color_rectangle_covered_map, cv::COLOR_GRAY2BGR);

    for (int y = 0; y < height; y++) {
        int start_x = -1;
        int end_x = -1;
        int end_y;
        for (int x = 0; x < width; x++) {
            if (rectangle_covered_map.at<uchar>(y, x) == 255) {
                if (start_x == -1) {
                    start_x = x;
                }
                end_x = x;
            } else {
                if (start_x != -1) {
                    end_y = y;
                    bool can_expand = true;
                    while (can_expand && end_y + 1 < height) {
                        for (int i = start_x; i < x; i++) {
                            if (rectangle_covered_map.at<uchar>(end_y + 1, i) != 255) {
                                can_expand = false;
                                break;
                            }
                        }
                        if (can_expand) end_y++;
                    }
                    cv::rectangle(rectangle_covered_map, {start_x, y}, {end_x, end_y}, cv::Scalar(0), -1);
                    cv::rectangle(color_rectangle_covered_map, {start_x, y}, {end_x, end_y}, cv::Scalar(255, 0 , 0), 1);
                    int rect_width = end_x - start_x + 1;
                    int rect_height = end_y - y + 1;
                    int rect_area = rect_width * rect_height;
                    m_cleaning_rect.push_back({start_x, y, rect_width, rect_height, rect_area});
                    RCLCPP_INFO(this->get_logger(), "Rect (%d, %d), (%d, %d) created", start_x, y, end_x, end_y);
                    start_x = -1;
                }
            }
        }
    }

    cv::imshow("map", color_rectangle_covered_map);
    cv::waitKey(10);
}

void CleanServer::costmap_callback(const OccupancyGrid::SharedPtr msg) {
    m_costmap = *msg;
    int width = m_cleaning_space.info.width;
    int height = m_cleaning_space.info.height;
    cv::Mat cost_map(height, width, CV_8UC1);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int8_t value = m_costmap.data[y * width + x];
            uint8_t pixel;

            if (value >= 20)
                pixel = 0;
            else
                pixel = 255;

            cost_map.at<uchar>(y, x) = pixel;
        }
    }
    cv::imshow("cost_map", cost_map);
    cv::waitKey(10);
}

rclcpp_action::GoalResponse CleanServer::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ActionCleanRoom::Goal> goal)
{
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CleanServer::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionCleanRoom>> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CleanServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionCleanRoom>> goal_handle)
{
    auto execute_in_thread = [this, goal_handle]()
    { return this->execute(goal_handle); };
    std::thread{execute_in_thread}.detach();
}

void CleanServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionCleanRoom>> goal_handle) {
    auto action_result = std::make_shared<ActionCleanRoom::Result>();
    int width = m_cleaning_space.info.width;
    int height = m_cleaning_space.info.height;

    if (m_robot_grid_pose.first < 0 || m_robot_grid_pose.first >= width ||
        m_robot_grid_pose.second < 0 || m_robot_grid_pose.second >= height) {
        RCLCPP_INFO(this->get_logger(), "No robot pose");
        action_result->result = ActionCleanRoom::Result::ABORTED;
        goal_handle->abort(action_result);
        return;
    }

    clean_goal_handle = goal_handle;

    std::vector<std::vector<std::pair<double, bool>>> dist_map(
        height, std::vector<std::pair<double, bool>>(width, {std::numeric_limits<double>::max(), false}));

    std::priority_queue<std::pair<double, std::pair<int, int>>, std::vector<std::pair<double, std::pair<int, int>>>, DistCompare> q;

    dist_map[m_robot_grid_pose.second][m_robot_grid_pose.first] = {0, false};

    q.push({0, m_robot_grid_pose});

    std::vector<std::pair<double, std::pair<int, int>>> dir = {
        {1, {1, 0}}, {1, {0, 1}}, {1, {-1, 0}}, {1, {0, -1}},
        {sqrt(2), {1, 1}}, {sqrt(2), {-1, 1}}, {sqrt(2), {-1, -1}}, {sqrt(2), {1, -1}}
    };
    RCLCPP_INFO(this->get_logger(), "Start get gains of %d rects", m_cleaning_rect.size());
    while(!q.empty()) {
        auto node = q.top();
        q.pop();
        int current_x = node.second.first;
        int current_y = node.second.second;

        if (dist_map[current_y][current_x].second)
            continue;
        dist_map[current_y][current_x].second = true;

        for (const auto& d : dir) {
            auto x = current_x + d.second.first;
            auto y = current_y + d.second.second;
            if (x < 0 || x >= width || y < 0 || y >= height || m_cleaning_space.data[y * width + x] == 100) continue;

            if (dist_map[y][x].first > dist_map[current_y][current_x].first + d.first) {
                dist_map[y][x].first = dist_map[current_y][current_x].first + d.first;
                if (dist_map[y][x].second == false) {
                    q.push({dist_map[y][x].first, {x, y}});
                }
            }
        }
    }

    int min_area = std::numeric_limits<int>::max();
    int max_area = std::numeric_limits<int>::lowest();
    double min_dist_all = std::numeric_limits<double>::max();
    double max_dist_all = std::numeric_limits<double>::lowest();

    for (auto &rect : m_cleaning_rect) {
        min_area = std::min(min_area, rect.area);
        max_area = std::max(max_area, rect.area);

        double min_dist = std::numeric_limits<double>::max();
        std::vector<std::pair<int, int>> points = {
            {rect.x, rect.y},
            {rect.x + rect.width - 1, rect.y},
            {rect.x, rect.y + rect.height - 1},
            {rect.x + rect.width - 1, rect.y + rect.height - 1}};

        for (auto [dx, dy] : points) {
            min_dist = std::min(min_dist, dist_map[dy][dx].first);
        }

        min_dist_all = std::min(min_dist_all, min_dist);
        max_dist_all = std::max(max_dist_all, min_dist);
    }

    auto normalize = [](double v, double vmin, double vmax)
    {
        if (vmax - vmin < 1e-6)
            return 0.0;
        return (v - vmin) / (vmax - vmin);
    };

    Rect *selected_rect = nullptr;
    double max_gain = -1.0;

    for (auto &rect : m_cleaning_rect) {
        double min_dist = std::numeric_limits<double>::max();

        std::vector<std::pair<int, int>> points = {
            {rect.x, rect.y},
            {rect.x + rect.width - 1, rect.y},
            {rect.x, rect.y + rect.height - 1},
            {rect.x + rect.width - 1, rect.y + rect.height - 1}};

        for (auto [dx, dy] : points) {
            min_dist = std::min(min_dist, dist_map[dy][dx].first);
        }

        double area_n = normalize(rect.area, min_area, max_area);
        double dist_n = normalize(min_dist, min_dist_all, max_dist_all);

        double gain = 0.3 * area_n + 0.7 * (1.0 - dist_n);

        if (gain > max_gain) {
            max_gain = gain;
            selected_rect = &rect;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Gains calculation is completed");

    current_rect = selected_rect;
    m_robot_state = RobotState::MOVING_TO_START;
    RCLCPP_INFO(this->get_logger(), "Current rect pose x : %d, y : %d", current_rect->x, current_rect->y);
    m_clean_timer->reset();
}

void CleanServer::pose_callback(const PoseWithCovarianceStamped::SharedPtr msg) {
    m_robot_grid_pose.first = (msg->pose.pose.position.x - m_cleaning_space.info.origin.position.x) / m_cleaning_space.info.resolution;
    m_robot_grid_pose.second = (msg->pose.pose.position.y - m_cleaning_space.info.origin.position.y) / m_cleaning_space.info.resolution;
}

void CleanServer::cleanRect() {

    static bool is_running = false;
    if (is_running) {
        return;
    }
    is_running = true;
    double resolution = m_costmap.info.resolution;
    int width = m_costmap.info.width;   
    std::vector<std::pair<int, int>> result_waypoints;

    RCLCPP_INFO(this->get_logger(), "Start creating path");

    bool is_target_left = (m_robot_state == RobotState::MOVING_TO_START ||
                            (m_robot_state == RobotState::MOVING_TO_NEXT_LINE && left_to_right) ||
                            (m_robot_state == RobotState::CLEANING_LINE && !left_to_right));
    
    for (int x = 0; x < current_rect->width; x++) {
        std::pair<int, int> pose;
        if (is_target_left) {
            pose.first = current_rect->x + x;
        } else {
            pose.first = current_rect->x + (current_rect->width - 1 - x);
        }
        pose.second = current_rect->y + current_y;

        bool is_free = m_costmap.data[pose.second * width + pose.first] < 20;

        if (is_free) {
            auto waypoints = astar(m_robot_grid_pose, pose, m_costmap);
            if (!waypoints.empty()) {
                for (auto& p : waypoints) {
                    result_waypoints.emplace_back(p);
                }
                break;
            }
        }
    }
    if (result_waypoints.empty())
    {
        is_running = false;
        current_y += y_step;
        if (current_rect->height <= current_y) {
            current_y = 0;
            checkCleanDone();
        }
        return;
    }
    

    
    if (follow_goal_handle != nullptr) {
        m_follow_client->async_cancel_goal(follow_goal_handle);
    }

    auto path = makePathFromWaypoints(result_waypoints);
    RCLCPP_INFO(this->get_logger(), "Finish creating path");
    m_path_pub->publish(path);
    sendFollowPath(path);
    is_running = false;
}

// void CleanServer::sendNavigateAction() {

//     NavigateToPose::Goal goal;

//     auto& info = m_cleaning_space.info;
//     double res = info.resolution;
//     double ox = info.origin.position.x;
//     double oy = info.origin.position.y;

//     auto [grid_x, grid_y] = waypoints.front();
//     waypoints.pop();

//     double yaw = std::atan2(grid_y -  m_robot_grid_pose.second, grid_x - m_robot_grid_pose.first);
//     tf2::Quaternion q;
//     q.setRPY(0, 0, yaw);

//     double x = ox + (grid_x + 0.5) * res;
//     double y = oy + (grid_y + 0.5) * res;

//     goal.pose.header.frame_id = "map";
//     goal.pose.header.stamp = now();
//     goal.pose.pose.position.x = x;
//     goal.pose.pose.position.y = y;
//     goal.pose.pose.orientation.x = q[0];
//     goal.pose.pose.orientation.y = q[1];
//     goal.pose.pose.orientation.z = q[2];
//     goal.pose.pose.orientation.w = q[3];

//     auto options = rclcpp_action::Client<
//         NavigateToPose>::SendGoalOptions();

//     options.result_callback =
//         std::bind(&CleanServer::onNavResult, this, std::placeholders::_1);

//     m_navigate_client->async_send_goal(goal, options);
// }

void CleanServer::onNavResult(
    const rclcpp_action::ClientGoalHandle<
        FollowPath>::WrappedResult &result) {
    follow_goal_handle.reset();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        switch (m_robot_state) {
            case RobotState::MOVING_TO_START:
            case RobotState::MOVING_TO_NEXT_LINE:
                m_robot_state = RobotState::CLEANING_LINE;
                break;
            case RobotState::CLEANING_LINE:
                current_y += y_step;
                m_robot_state = RobotState::MOVING_TO_NEXT_LINE;
                left_to_right = !left_to_right;
                if (current_rect->height <= current_y) {
                    is_rect_cleaned = true;
                    current_y = 0;
                    left_to_right = true;
                    m_robot_state = MOVING_TO_START;
                }
                break;
        }
            
    }
    if (!is_rect_cleaned) {
        return;
    }
    checkCleanDone();
}

std::vector<std::pair<int, int>> CleanServer::astar(std::pair<int, int> start,
      std::pair<int, int> goal,
      const OccupancyGrid& map)
{
    int width = map.info.width;
    int height = map.info.height;

    auto isRobotArea = [this](int x, int y) -> bool {
        int dx = x - m_robot_grid_pose.first;
        int dy = y - m_robot_grid_pose.second;
        return (dx * dx + dy * dy) <= pow(0.3 / 0.05, 2);
    };

    auto hash = [&](int x, int y)
    {
        return y * width + x;
    };

    auto heuristic = [&](int x, int y)
    {
        return std::hypot(x - goal.first, y - goal.second) + abs(y - (current_rect->y + current_y));
    };

    struct Node
    {
        int x, y;
        float g, h;
        int parent; // index
        float f() const { return g + h; }
    };

    struct NodeCompare {
        bool operator()(const Node &a, const Node &b) const
        {
            return a.f() > b.f();
        }
    };

    std::priority_queue<Node, std::vector<Node>, NodeCompare> open;

    std::unordered_map<int, Node> nodes;
    std::unordered_set<int> closed;

    int start_idx = hash(start.first, start.second);
    nodes[start_idx] = {
        start.first,
        start.second,
        0.0f,
        heuristic(start.first, start.second),
        -1};

    open.push(nodes[start_idx]);

    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open.empty())
    {
        Node current = open.top();
        open.pop();

        int cur_idx = hash(current.x, current.y);
        if (closed.count(cur_idx))
            continue;

        if (current.x == goal.first && current.y == goal.second)
        {
            std::vector<std::pair<int, int>> path;
            int idx = cur_idx;

            while (idx != -1)
            {
                auto &n = nodes[idx];
                path.emplace_back(n.x, n.y);
                idx = n.parent;
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        closed.insert(cur_idx);

        for (int i = 0; i < 8; i++)
        {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                continue;

            int nidx = hash(nx, ny);
            if (closed.count(nidx))
                continue;

            if (map.data[nidx] >= 20 && !isRobotArea(nx, ny))
                continue;

            float tentative_g = current.g + 1.0f;

            if (!nodes.count(nidx) || tentative_g < nodes[nidx].g)
            {
                nodes[nidx] = {
                    nx,
                    ny,
                    tentative_g,
                    heuristic(nx, ny),
                    cur_idx};
                open.push(nodes[nidx]);
            }
        }
    }

    return {};
}

nav_msgs::msg::Path CleanServer::makePathFromWaypoints(const std::vector<std::pair<int, int>> &waypoints) const {
    nav_msgs::msg::Path path;
    path.header.frame_id = m_costmap.header.frame_id;
    // path.header.stamp = this->now();

    double res = m_costmap.info.resolution;
    double ox = m_costmap.info.origin.position.x;
    double oy = m_costmap.info.origin.position.y;

    for (size_t i = 0; i < waypoints.size(); i+=1)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;

        int gx = waypoints[i].first;
        int gy = waypoints[i].second;

        pose.pose.position.x = ox + (gx + 0.5) * res;
        pose.pose.position.y = oy + (gy + 0.5) * res;
        pose.pose.position.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Path x : %f, y : %f", pose.pose.position.x, pose.pose.position.y);

        if (i + 1 < waypoints.size())
        {
            int nx = waypoints[i + 1].first;
            int ny = waypoints[i + 1].second;
            double yaw = std::atan2(ny - gy, nx - gx);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);

            pose.pose.orientation.x = q[0];
            pose.pose.orientation.y = q[1];
            pose.pose.orientation.z = q[2];
            pose.pose.orientation.w = q[3];
        }
        else
        {
            pose.pose.orientation.w = 1.0;
        }

        path.poses.push_back(pose);
    }

    return path;
}

void CleanServer::sendFollowPath(const nav_msgs::msg::Path &path) {

    nav2_msgs::action::FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = "FollowPath";

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](auto handle) {
        follow_goal_handle = handle;
    };

    send_goal_options.result_callback =
        std::bind(&CleanServer::onNavResult, this, std::placeholders::_1);

    m_follow_client->async_send_goal(goal, send_goal_options);
}

void CleanServer::checkCleanDone() {
    m_clean_timer->cancel();
    if (follow_goal_handle != nullptr) {
        m_follow_client->async_cancel_goal(follow_goal_handle);
    }
    m_cleaning_rect.erase(
        std::remove_if(
            m_cleaning_rect.begin(),
            m_cleaning_rect.end(),
            [&](const Rect &r)
            { return &r == current_rect; }),
        m_cleaning_rect.end());

    current_rect = nullptr;
    is_rect_cleaned = false;

    if (!m_cleaning_rect.empty())
    {
        m_clean_timer->reset();
        return;
    }
    m_robot_state = RobotState::IDLE;

    auto res = std::make_shared<ActionCleanRoom::Result>();
    res->result = ActionCleanRoom::Result::SUCCESS;
    clean_goal_handle->succeed(res);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CleanServer>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}