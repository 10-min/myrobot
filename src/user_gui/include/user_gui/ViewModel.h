#ifndef VIEW_MODEL_H_
#define VIEW_MODEL_H_

#include <QObject>
#include <vector>
#include <QImage>
#include <memory>

class ViewModel : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QImage map READ map NOTIFY mapChanged)
    Q_PROPERTY(int mapWidth READ mapWidth NOTIFY mapChanged)
    Q_PROPERTY(int mapHeight READ mapHeight NOTIFY mapChanged)

private:
    QImage m_map;

    QImage map() const;
    int mapWidth() const;
    int mapHeight() const;

public:
    explicit ViewModel(QObject* parent = nullptr);
    Q_INVOKABLE void onCleanRoom();

public slots:
    void onUpdateMap(QImage map);

signals:
    void mapChanged();
    void cleanRoom();
};

#endif