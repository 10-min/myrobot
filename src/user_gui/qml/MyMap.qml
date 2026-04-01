import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import Autobot 1.0

Image {
    source: idViewModel.map

    Canvas {
        id: overlay
        anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)

            ctx.beginPath()
            ctx.moveTo(100, 100)
            ctx.lineTo(200, 100)
            ctx.lineTo(150, 200)
            ctx.closePath()

            ctx.fillStyle = "rgba(255, 0, 0, 0.5)" // 반투명 빨강
            ctx.fill()

            ctx.strokeStyle = "red"
            ctx.lineWidth = 2
            ctx.stroke()
        }
    }

    MouseArea {
        anchors.fill: parent

        onClicked: {
            console.info("x : ", mouseX, "y : ", mouseY)
        }
    }

    ViewModel {
        id: idViewModel
    }

    Repeater {
        model: idViewModel.polygon_model
        delegate: Item {
            Component.onCompleted: {
                console.info(model.name)
            }
        }
    }
}
