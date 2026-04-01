#include "user_gui/PlacePolygon.h"
#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <QString>
#include <QPointF>

PlacePolygon::PlacePolygon(QObject* parent) : QAbstractListModel(parent) {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("user_gui");
}

int PlacePolygon::rowCount(const QModelIndex& parent) const {
    if (parent.isValid()) return 0;
    return m_items.size();
}

QVariant PlacePolygon::data(const QModelIndex& index, int role) const {
    const auto &item = m_items[index.row()];

    switch (role)
    {
    case NameRole:
        return item.name;
    case PolygonRole:
        return item.points;
    
    default:
        break;
    }
    QVariant();
}

QHash<int, QByteArray> PlacePolygon::roleNames()  const {
    return {
        { NameRole, "name" },
        { PolygonRole, "points"}
    };
}

void PlacePolygon::addPolygon(const QString &name, const QVariantList &points) {
    beginInsertRows(QModelIndex(), m_items.size(), m_items.size());
    m_items.append(PolygonData{name, points});
    endInsertRows();
}