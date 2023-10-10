import QtQuick 2.0
import QtPositioning 5.15
import QtLocation 5.15

Rectangle {
    id: window // main widget window id
    // component maker
    property Component droneMaker: droneMarker
    property Component pinMaker: pinMarker
    property Component destMaker: destinationMarker
    // map items
    property var drones: [undefined, undefined, undefined, undefined,
                            undefined, undefined, undefined, undefined]
    property var destinations: [undefined, undefined, undefined, undefined, undefined,
                            undefined, undefined, undefined, undefined]
    property var pin: undefined
    // other variables
    property int numDrones: 8
    property int centerDrone: 1
    property int zoom: 30
    property int droneSize: 1
    property bool autoUpdate: true
    property bool selectingDestination: false
    property double centerLat: 32.209873
    property double centerLon: -110.922644

    /* Instantiate a Plugin instance. We're using OSM plugin here. */
    Plugin {
        id: mapPlugin
        name: "osm"
    }

    /*
     * Map type to display the map. MouseArea to help the cursor model
     */
    Map {
        id: mapView
        objectName: "mapView"
        anchors.fill: parent
        plugin: mapPlugin
        center: QtPositioning.coordinate(centerLat, centerLon)
        zoomLevel: zoom

        // Component.onCompleted: addDrone(5, centerLat, centerLon, 0, "5")

        // MouseArea {
        //     anchors.fill: parent
        //     hoverEnabled: true
        //     cursorShape: selectingDestination ? Qt.CrossCursor : Qt.ArrowCursor
        // }
    }

    /* drone component */
    Component {
        id: droneMarker
        MapQuickItem {
            id: drone
            property int bearing: 0
            property string droneNumber: droneNumberText
            property string color: colorText

            NumberAnimation on rotation { from: bearing; to: bearing; duration: 10; }

            anchorPoint.x: droneImage.width/2
            anchorPoint.y: droneImage.height/2
            coordinate: position
            sourceItem: Image {
                // rotation: bearing
                id: droneImage

                source: "file:///C:/Users/Draco/OneDrive/AQUABOT/icon/drone-" + color + ".png"

                width: 8.75 * droneSize
                height: 25 * droneSize

                Text {
                    id: droneText
                    text: droneNumber
                    font.pixelSize: 9 + droneSize
                    color: "white"
                    anchors.fill: parent
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
            }
        }
    }

    /* pin component */
    Component {
        id: pinMarker
        MapQuickItem {
            id: pin

            anchorPoint.x: pinImage.width/4
            anchorPoint.y: pinImage.height
            coordinate: position
            sourceItem: Image {
                id: pinImage
                source: "file:///C:/Users/Draco/OneDrive/AQUABOT/icon/pin-red.png"
            }
        }
    }

    /* destination component */
    Component {
        id: destinationMarker
        MapQuickItem {
            id: destination
            property string whose: whoseText
            anchorPoint.x: destinationImage.width/4
            anchorPoint.y: destinationImage.height
            coordinate: position
            sourceItem: Grid {
                columns: 1
                Image {
                    id: destinationImage
                    source: "file:///C:/Users/Draco/OneDrive/AQUABOT/icon/pin-green.png"
                }
                Text {
                    id: destText
                    text: whose
                    font.pixelSize: 12
                }
            }
        }
    }

    MouseArea {
        anchors.fill: parent
        property int lastX: -1
        property int lastY: -1

        onDoubleClicked:  {
            if (selectingDestination) {
                var coord = mapView.toCoordinate(Qt.point(mouse.x,mouse.y));
                if (pin !== undefined) mapView.removeMapItem(pin);
                pin = pinMaker.createObject(window,{coordinate: coord});
                mapView.addMapItem(pin);
            }
        }

        /* The following is to enable the drag */ 
        onPressed : {
            lastX = mouse.x
            lastY = mouse.y
        }

        onPositionChanged: {
            mapView.pan(lastX-mouse.x, lastY-mouse.y)
            lastX = mouse.x
            lastY = mouse.y
        }
    }

    function changeCenter(droneNum) { centerDrone = droneNum; }

    function updateDroneSize(newSize) {
        droneSize = newSize;

        for (let i = 0; i < numDrones; i++) {
            if (drones[i] !== undefined) {
                var coord = drones[i].coordinate;
                var bearing = drones[i].bearing;
                var num = drones[i].droneNumber;
                var color = drones[i].color;
                mapView.removeMapItem(drones[i]);
                drones[i] = droneMaker.createObject(window,
                                    {coordinate: coord,
                                        bearing: bearing,
                                        droneNumber: num,
                                        color: color}
                                    );
                mapView.addMapItem(drones[i]);
            }
        }
    }

    /* arg as 0 or 2 */
    function setAutoUpdate(arg) {
        if (arg === 2) autoUpdate = true;
        else if (arg === 0) autoUpdate = false;
    }

    function setSelectingDestination(arg: bool) { selectingDestination = arg; }

    function removePin() {
        mapView.removeMapItem(pin);
        pin = undefined;
    }

    function updateDrone(droneNum: int, currLat: double, currLon: double, bearing: double, inSwarm: bool) {
        if (drones[droneNum - 1] !== undefined)
            mapView.removeMapItem(drones[droneNum - 1]);
        addDrone(droneNum, currLat, currLon, bearing, droneNum.toString(), inSwarm);
        if (autoUpdate && centerDrone === droneNum) {
            mapView.pan(centerLat - currLat, centerLon - currLon);
            centerLat = currLat;
            centerLon = currLon;
        }
    }

    function addDrone(droneNum, currLat, currLon, bearing, text, inSwarm) {
        var color = "black";
        if (!inSwarm) color = "navy";
        drones[droneNum - 1] = droneMaker.createObject(window,
                                    {coordinate: QtPositioning.coordinate(currLat, currLon),
                                        bearing: bearing,
                                        droneNumber: text,
                                        color: color});
        mapView.addMapItem(drones[droneNum - 1]);
    }

    function setDest(who, lat, lon) {
        var index;
        var text = "";
        if (who === "Swarm") index = 0;
        else {
            index = who;
            text = who;
        }
        if (destinations[index] !== undefined)
            mapView.removeMapItem(destinations[index]);

        destinations[index] = destMaker.createObject(window,
                                                     {coordinate: QtPositioning.coordinate(lat, lon),
                                                      whose: text});
        mapView.addMapItem(destinations[index]);
    }

    function removeDest(who) {
        var index;
        if (who === "Swarm") index = 0;
        else index = who;

        mapView.removeMapItem(destinations[index]);
        destinations[index] = undefined;
    }

}
