#ifndef DRONE_H
#define DRONE_H

#include <QDateTime>
#include <QDebug>
#include <QGeoCoordinate>

class Drone {

public:
    /* Constructor */
    Drone();
    /* getter */
    int getDroneNumber();
    QGeoCoordinate getLocation();
    QGeoCoordinate getIndivDestination();
    QGeoCoordinate getSwarmDestination();
    QDateTime getLastModifiedTime();
    /* setter */
    void setActivated(bool flag);
    void setDroneNumber(int num);
    void setStarted(bool flag);
    void setInSwarm(bool flag);
    // void setMoving(bool flag);
    void setLocation(float lat, float lon);
    void setIndivDestination(float lat, float lon);
    void setSwarmDestination(float lat, float lon);
    void setLastModifiedTime(QDateTime newTime);
    /* Other */
    bool isActivated();
    bool isStarted();
    bool isInSwarm();
    // bool isMoving();

private:
    bool activated;
    int droneNumber;
    bool started;
    bool inSwarm;
    // bool moving;
    QGeoCoordinate location;
    QGeoCoordinate indivDestination;
    QGeoCoordinate swarmDestination;
    QDateTime lastModified;

};

#endif // DRONE_H
