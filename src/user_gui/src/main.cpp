#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <string>
#include <QQmlContext>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "user_gui/ViewModel.h"
#include "user_gui/RosWorker.h"
#include "user_gui/PlacePolygon.h"
#include "user_gui/MapPaintItem.h"

int main(int argc, char* argv[]) {
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    std::string share_dir = ament_index_cpp::get_package_share_directory("user_gui");
    QString qmlPath = QString::fromStdString(share_dir + "/qml/main.qml");
    ViewModel* vm = new ViewModel();
    RosWorker* worker = new RosWorker();

    worker->start();

    QObject::connect(worker, &RosWorker::mapReceived,
                     vm, &ViewModel::onUpdateMap, Qt::QueuedConnection);
    QObject::connect(vm, &ViewModel::cleanRoom,
                     worker, &RosWorker::cleanRoom, Qt::QueuedConnection);
    engine.rootContext()->setContextProperty("ViewModel", vm);
    qmlRegisterType<MapPaintItem>("Myrobot", 1, 0, "MapPaintItem");
    engine.load(QUrl::fromLocalFile(qmlPath));
    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}