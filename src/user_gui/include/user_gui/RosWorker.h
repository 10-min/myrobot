#ifndef ROS_WORKER_H
#define ROS_WORKER_H

#include "rclcpp/rclcpp.hpp"
#include "user_gui/ViewModel.h"
#include "user_gui/ManagerModel.h"

#include <QThread>
#include <QImage>

class RosWorker : public QThread {
    Q_OBJECT
private:
    std::shared_ptr<ManagerModel> node;

    void onMapReceived(QImage map);
public:
    explicit RosWorker(QObject *parent = nullptr);
    ~RosWorker() {}

    void cleanRoom();

protected:
    void run() override;

signals:
    void mapReceived(QImage map);

};

#endif