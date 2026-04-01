#include "user_gui/ViewModel.h"
#include "user_gui/PlacePolygon.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <QDebug>

ViewModel::ViewModel(QObject *parent) 
    : QObject(parent) {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("user_gui");
    m_map = QImage();

}

QImage ViewModel::map() const {
    return m_map;
}

int ViewModel::mapWidth() const {
    return m_map.width();
}

int ViewModel::mapHeight() const {
    return m_map.height();
}

void ViewModel::onCleanRoom() {
    emit cleanRoom();
}

void ViewModel::onUpdateMap(QImage map) {
    QMetaObject::invokeMethod(
        this,
        [this, map]()
        {
            m_map = map;
            emit mapChanged();
        },
        Qt::QueuedConnection);
}