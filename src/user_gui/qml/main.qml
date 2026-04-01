import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import Myrobot 1.0

Window {
    id: idWindow
    width: 600
    height: 800
    visible: true
    title: "Autobot Navigation"
    Rectangle {
        id: idMapArea
        width: ViewModel.mapWidth
        height: ViewModel.mapHeight
        anchors.horizontalCenter: parent.horizontalCenter
        color: "#f0f0f0"


        MapPaintItem {
            anchors.fill: parent
            mapImage: ViewModel.map
            onMapImageChanged: {
                console.info(ViewModel.mapWidth);
            }
        }
    }

    Rectangle {
        width: parent.width
        anchors.top: idMapArea.bottom
        anchors.bottom: parent.bottom
        
        Button {
            id: btnCreateMap
            text: "Create Map"
        }
        Button {
            id: btnCleanRoom
            anchors.top: btnCreateMap.bottom
            text: "Clean Room"
            onClicked: {
                ViewModel.onCleanRoom();
            }
        }
    }
    
}
