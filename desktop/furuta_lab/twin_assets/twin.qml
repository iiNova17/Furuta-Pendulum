// Furuta Lab digital twin (Qt Quick 3D). Units: mm, world frame Z up (the CAD frame), turned to Qt's Y-up by `world`.
// The poses are set from Python (twin_view.py) on every frame: armJoint / pendJoint and the ghost copies.
import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import FurutaTwin 1.0

Item {
    id: root

    View3D {
        id: view
        anchors.fill: parent
        environment: SceneEnvironment {
            clearColor: "#e7ecf1"
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        Node {
            id: camOrigin
            objectName: "camOrigin"
            position: Qt.vector3d(0, 105, 0)
            eulerRotation: Qt.vector3d(-24, 38, 0)
            PerspectiveCamera {
                id: cam
                objectName: "camera"
                position: Qt.vector3d(0, 0, 440)
                fieldOfView: 35
                clipNear: 2
                clipFar: 8000
            }
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-55, -35, 0)
            brightness: 1.15
            castsShadow: true
            shadowMapQuality: Light.ShadowMapQualityHigh
            shadowFactor: 30
        }
        DirectionalLight { eulerRotation: Qt.vector3d(-15, 150, 0); brightness: 0.45 }
        DirectionalLight { eulerRotation: Qt.vector3d(70, 20, 0); brightness: 0.25 }

        Node {
            id: world
            eulerRotation: Qt.vector3d(-90, 0, 0)          // CAD frame (Z up) -> Qt (Y up)

            Model {
                source: "#Rectangle"
                scale: Qt.vector3d(9, 9, 1)                 // 900 mm floor in the XY plane at z = 0
                receivesShadows: true
                materials: PrincipledMaterial { baseColor: "#d3d9df"; roughness: 0.95 }
            }
            Model {
                objectName: "housing"
                geometry: RipGeometry { source: "housing" }
                castsShadows: true
                receivesShadows: true
                materials: PrincipledMaterial { baseColor: "#8c959e"; roughness: 0.55; metalness: 0.1 }
            }
            Node {
                id: armJoint
                objectName: "armJoint"
                Model {
                    geometry: RipGeometry { source: "arm" }
                    castsShadows: true
                    receivesShadows: true
                    materials: PrincipledMaterial { baseColor: "#1c1d21"; roughness: 0.5 }
                }
                Node {
                    objectName: "pendJoint"
                    Model {
                        objectName: "pendModel"
                        geometry: RipGeometry { source: "pendulum" }
                        castsShadows: true
                        receivesShadows: true
                        materials: PrincipledMaterial { baseColor: "#c62828"; roughness: 0.4 }
                    }
                }
            }
            Node {
                objectName: "ghostArm"
                visible: false
                Model {
                    geometry: RipGeometry { source: "arm" }
                    materials: PrincipledMaterial { baseColor: "#38d6ff"; opacity: 0.3; alphaMode: PrincipledMaterial.Blend; roughness: 0.3 }
                }
                Node {
                    objectName: "ghostPend"
                    Model {
                        objectName: "ghostPendModel"
                        geometry: RipGeometry { source: "pendulum" }
                        materials: PrincipledMaterial { baseColor: "#38d6ff"; opacity: 0.3; alphaMode: PrincipledMaterial.Blend; roughness: 0.3 }
                    }
                }
            }
        }
    }

    OrbitCameraController {
        anchors.fill: view
        origin: camOrigin
        camera: cam
    }

    Rectangle {
        x: 10; y: 10
        width: hud.implicitWidth + 16; height: hud.implicitHeight + 12
        radius: 6
        color: "#cc263649"
        Text {
            id: hud
            objectName: "hud"
            x: 8; y: 6
            color: "white"
            font.family: "Consolas"
            font.pixelSize: 13
            text: "Digital twin"
        }
    }
    Text {
        anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.margins: 8
        color: "#6b7785"; font.pixelSize: 11
        text: "drag: orbit  ·  wheel: zoom  ·  solid = rig, cyan = model  ·  pend 0° = upright"
    }
}
