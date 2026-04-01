#include "user_gui/ManagerModel.h"
#include <QPainter>

ManagerModel::ManagerModel() : Node("manager_model") {
    m_map_sub = this->create_subscription<OccupancyGrid>(
        "map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&ManagerModel::mapCallback, this, std::placeholders::_1)
    );

    m_robot_pose_sub = this->create_subscription<PoseWithCovarianceStamped>(
        "amcl_pose",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&ManagerModel::poseCallback, this, std::placeholders::_1)
    );

    action_clean_room_client = rclcpp_action::create_client<ActionCleanRoom> (
        this, "/clean_room"
    );

    m_robot_pose = {0, 0};

}

void ManagerModel::mapCallback(const OccupancyGrid::SharedPtr msg) {
    m_map = *msg;
    m_map_image = occupancyGridToImage(msg);
    QImage map_with_robot_image = m_map_image;
    int grid_x = (m_robot_pose.first - m_map.info.origin.position.x) / m_map.info.resolution;
    int grid_y = (m_robot_pose.second - m_map.info.origin.position.y) / m_map.info.resolution;
    paintRobotPose(map_with_robot_image, {grid_x, grid_y});
    map_handler(map_with_robot_image);
}

QImage ManagerModel::occupancyGridToImage(const OccupancyGrid::SharedPtr grid) {
    int width = grid->info.width;
    int height = grid->info.height;

    QImage image(width, height, QImage::Format_RGB888);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {

            int i = x + y * width;
            int value = grid->data[i];

            int color;

            if (value == -1)
            { // unknown
                color = 128;
            }
            else
            { // 0~100
                color = 255 - (value * 255 / 100);
            }

            image.setPixel(x, height - 1 - y, qRgb(color, color, color));
        }
    }

    return image;
}

void ManagerModel::poseCallback(const PoseWithCovarianceStamped::SharedPtr msg) {
    auto position = msg->pose.pose.position;
    auto x = position.x;
    auto y = position.y;
    m_robot_pose = {x, y};
    RCLCPP_INFO(this->get_logger(), "Robot pose received, x: %f, y: %f", x, y);

    if (m_map.info.width <= 0 || m_map.info.height <= 0) {
        RCLCPP_INFO(this->get_logger(), "No map received, do nothing");
    }

    int grid_x = (x - m_map.info.origin.position.x) / m_map.info.resolution;
    int grid_y = (y - m_map.info.origin.position.y) / m_map.info.resolution;
    QImage map_with_robot_image = m_map_image;
    paintRobotPose(map_with_robot_image, {grid_x, grid_y});
    map_handler(map_with_robot_image);
}

void ManagerModel::paintRobotPose(QImage &map_image, std::pair<int, int> map_pose) {
    QPainter painter(&map_image);
    painter.setRenderHint(QPainter::Antialiasing);

    QPoint q_pose = QPoint(map_pose.first, map_pose.second);

    painter.setPen(QPen(Qt::blue, 3));
    painter.setBrush(QBrush(QColor(0, 0, 255, 150)));
    painter.drawEllipse(q_pose, 8, 8);
}

void ManagerModel::cleanRoom() {
    
    ActionCleanRoom::Goal goal;

    rclcpp_action::Client<ActionCleanRoom>::SendGoalOptions options;

    options.result_callback = [this](const rclcpp_action::ClientGoalHandle<ActionCleanRoom>::WrappedResult &result) {
        bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        RCLCPP_INFO(this->get_logger(), "Succeed cleaning room");
    };

    action_clean_room_client->async_send_goal(goal, options);
}