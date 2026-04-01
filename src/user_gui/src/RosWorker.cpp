#include "user_gui/RosWorker.h"
#include <QDebug>

RosWorker::RosWorker(QObject *parent) : QThread(parent) {
}

void RosWorker::run() {
    rclcpp::init(0, nullptr);
    node = std::make_shared<ManagerModel>();

    node->map_handler = std::bind(&RosWorker::onMapReceived, this, std::placeholders::_1);

    rclcpp::spin(node);
    rclcpp::shutdown();
}

void RosWorker::onMapReceived(QImage map) {
    qDebug() << "Map Callback";
    emit mapReceived(map);
}

void RosWorker::cleanRoom() {
    node->cleanRoom();
}
