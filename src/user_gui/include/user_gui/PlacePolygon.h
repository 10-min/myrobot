#ifndef PLACE_POLYGON_H_
#define PLACE_POLYGON_H_

#include <string>
#include <QAbstractListModel>

class PlacePolygon : public QAbstractListModel {
    Q_OBJECT
public:
    enum Roles {
        NameRole = Qt::UserRole + 1,
        PolygonRole
    };

    struct PolygonData {
        QString name;
        QVariantList points;
    };

    explicit PlacePolygon(QObject* parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;
    void addPolygon(const QString& name, const QVariantList& points);

private:
    QList<PolygonData> m_items;
};

#endif