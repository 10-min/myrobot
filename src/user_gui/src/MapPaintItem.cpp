#include "user_gui/MapPaintItem.h"
#include <QPainter>

MapPaintItem::MapPaintItem(QQuickItem *parent)
    : QQuickPaintedItem(parent)
{
}

QImage MapPaintItem::mapImage() const
{
    return map_;
}

void MapPaintItem::setMapImage(const QImage &image)
{
    qDebug() << "setMapImage() called size " << image.width() << " x " << image.height();
    map_ = image;
    emit mapImageChanged();
    update(); // repaint

    QMetaObject::invokeMethod(
        this,
        "update",
        Qt::QueuedConnection);
}

void MapPaintItem::paint(QPainter *painter)
{
    qDebug() << "paint() called";
    if (map_.isNull())
        return;

    painter->drawImage(boundingRect(), map_);
}
