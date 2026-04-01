#include "place_nav/map_creator.hpp"
#include <cmath>

MapCreator::MapCreator(): Node("map_creator"), robot_map_x(0), robot_map_y(0), m_is_get_robot_pose(false) {
    m_map_sub = this->create_subscription<OccupancyGrid>(
        "/map",
        10,
        std::bind(&MapCreator::map_callback, this, std::placeholders::_1));

    m_goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    m_pose_sub = this->create_subscription<PoseWithCovarianceStamped>(
        "/pose",
        10,
        std::bind(&MapCreator::pose_callback, this, std::placeholders::_1));
}

MapCreator::~MapCreator() {

}

void MapCreator::map_callback(const OccupancyGrid::SharedPtr msg)
{
    int width = msg->info.width;
    int height = msg->info.height;

    if (width <= 0 || height <= 0) return;

    m_map = *msg.get();

    double robot_radius_m = 0.20; // 로봇 반경(미터)
    int inflate_pixels = robot_radius_m / m_map.info.resolution;

    // original occupancy to cv::Mat
    cv::Mat occ(height, width, CV_8UC1);
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int8_t v = m_map.data[y * width + x];
            occ.at<uchar>(y, x) = (v == CellValue::OCCUPIED ? 255 : 0); // obstacle만 흰색(255)
        }
    }

    // dilation kernel (로봇 반지름 반영)
    int k = inflate_pixels * 2 + 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));

    // inflate 실행
    cv::Mat inflated;
    cv::dilate(occ, inflated, kernel);

    // inflated 결과를 occupancyGrid에 반영
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int idx = y * width + x;

            if (inflated.at<uchar>(y, x) == 255)
            {
                m_map.data[idx] = CellValue::OCCUPIED; // inflate로 장애물 확장
            }
        }
    }

    if (!m_is_get_robot_pose) {
        robot_map_x = (0.0 - m_map.info.origin.position.x) / m_map.info.resolution;
        robot_map_y = (0.0 - m_map.info.origin.position.y) / m_map.info.resolution;
    }

    cv::Mat img(height, width, CV_8UC1);
    const std::vector<int8_t> &data = m_map.data;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int8_t value = data[y * width + x];
            uint8_t pixel;

            switch (value) {
                case -1:
                    pixel = 100;
                    break;
                case 100:
                    pixel = 0;
                    break;
                default:
                    pixel = 255;
            }
            // RCLCPP_INFO(this->get_logger(), "%d", value);
            inflated.at<uchar>(y, x) = pixel;
        }
    }

    auto frontiers = detect_frontiers();
    if (frontiers.empty())
    {
        RCLCPP_INFO(get_logger(), "No reachable frontier found.");
        return;
    }

    auto clusters = cluster_frontiers(frontiers);
    auto goal = choose_best_cluster(clusters);

    m_goal_pub->publish(goal);

    cv::resize(inflated, inflated, cv::Size(width * 2, height * 2));

    cv::imshow("Costmap", inflated);
    cv::waitKey(1);
}

void MapCreator::pose_callback(const PoseWithCovarianceStamped::SharedPtr msg) {
    m_is_get_robot_pose = true;
    robot_map_x = (msg->pose.pose.position.x - m_map.info.origin.position.x) / m_map.info.resolution;
    robot_map_y = (msg->pose.pose.position.y - m_map.info.origin.position.y) / m_map.info.resolution;
}

double MapCreator::dist(auto &a, auto &b) const {
    return hypot(a.first - b.first, a.second - b.second);
}

int MapCreator::xy_to_idx(int x, int y) const {
    return y * m_map.info.width + x;
}

bool MapCreator::is_valid(int x, int y) const {
    return x >= 0 && y >= 0 && x < m_map.info.width && y < m_map.info.height;
}

std::vector<std::pair<int, int>> MapCreator::neighbors(int x, int y) const {
    return {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
}

bool MapCreator::is_frontier_cell(int x, int y) const {
    for (int dy = -1; dy <= 1; dy++)
    {
        for (int dx = -1; dx <= 1; dx++)
        {
            if (dx == 0 && dy == 0)
                continue;

            int nx = x + dx;
            int ny = y + dy;

            if (!is_valid(nx, ny))
                continue;

            int idx = xy_to_idx(nx, ny);
            
            if (m_map.data[idx] == CellValue::UNKNOWN)
                return true;
        }
    }
    return false;
}

std::vector<std::pair<int, int>> MapCreator::detect_frontiers() const {
    std::vector<std::pair<int, int>> frontiers;

    std::vector<bool> visited(m_map.data.size(), false);
    std::queue<std::pair<int, int>> q;

    q.push({robot_map_x, robot_map_y});
    visited[xy_to_idx(robot_map_x, robot_map_y)] = true;

    while (!q.empty())
    {
        auto [x, y] = q.front();
        q.pop();
        int idx = xy_to_idx(x, y);
        if (m_map.data[idx] == CellValue::FREE && is_frontier_cell(x, y))
            frontiers.push_back({x, y});

        for (auto [nx, ny] : neighbors(x, y))
        {
            if (!is_valid(nx, ny))
                continue;
            int nidx = xy_to_idx(nx, ny);
            if (visited[nidx])
                continue;

            if (m_map.data[nidx] == CellValue::FREE) { 
                visited[nidx] = true;
                q.push({nx, ny});
            }
        }
    }
    return frontiers;
}

std::vector<std::vector<std::pair<int, int>>> MapCreator::cluster_frontiers(
    const std::vector<std::pair<int, int>>& frontiers
) const {
    std::vector<std::vector<std::pair<int, int>>> clusters;
    std::vector<bool> visited(frontiers.size(), false);

    for (size_t i = 0; i < frontiers.size(); i++) {
        if (visited[i])
            continue;
        std::queue<int> q;
        std::vector<std::pair<int, int>> c;

        q.push(i);
        visited[i] = true;

        while (!q.empty()) {
            auto idx = q.front();
            q.pop();
            c.push_back(frontiers[idx]);

            for (size_t j = 0; j < frontiers.size(); j++) {
                if (!visited[j] && dist(frontiers[idx], frontiers[j]) < 3.0) {
                    visited[j] = true;
                    q.push(j);
                }
            }
        }
        clusters.push_back(c);
    }
    return clusters;
}

geometry_msgs::msg::PoseStamped MapCreator::choose_best_cluster(const std::vector<std::vector<std::pair<int, int>>> &clusters) const {
    if (clusters.empty()) {
        RCLCPP_WARN(get_logger(), "No frontier clusters found.");
        return geometry_msgs::msg::PoseStamped();
    }

    double best_score = -1;
    std::vector<std::pair<int, int>> best_cluster;

    for (const auto &cluster : clusters) {
        if (cluster.empty())
            continue;

        double sx = 0, sy = 0;
        for (auto &p : cluster)
        {
            sx += p.first;
            sy += p.second;
        }
        sx /= cluster.size();
        sy /= cluster.size();

        double dist_from_robot = hypot(sx - robot_map_x, sy - robot_map_y);

        double score = cluster.size() / (dist_from_robot + 1.0);

        if (score > best_score)
        {
            best_score = score;
            best_cluster = cluster;
        }
    }

    double sx = 0, sy = 0;
    for (auto &p : best_cluster) {
        sx += p.first;
        sy += p.second;
    }

    sx /= best_cluster.size();
    sy /= best_cluster.size();

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = sx * m_map.info.resolution + m_map.info.origin.position.x;
    goal.pose.position.y = sy * m_map.info.resolution + m_map.info.origin.position.y;
    goal.pose.orientation.w = 1.0;
    return goal;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = std::make_shared<MapCreator>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();

    return 0;
}