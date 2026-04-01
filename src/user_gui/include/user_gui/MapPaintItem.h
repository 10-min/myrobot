#pragma once

#include <QQuickPaintedItem>
#include <QImage>

class MapPaintItem : public QQuickPaintedItem
{
    Q_OBJECT
    Q_PROPERTY(QImage mapImage READ mapImage WRITE setMapImage NOTIFY mapImageChanged)

public:
    explicit MapPaintItem(QQuickItem *parent = nullptr);
    ~MapPaintItem() {}

    QImage mapImage() const;
    void setMapImage(const QImage &image);

    void paint(QPainter *painter) override;

signals:
    void mapImageChanged();

private:
    QImage map_;
};
