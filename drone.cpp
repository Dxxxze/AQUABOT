#include "drone.h"

Drone::Drone()
{
    activated = false;
    started = false;
    inSwarm = true;
    // moving = false;
}

/* Getter */

int Drone::getDroneNumber() { return droneNumber; }

QGeoCoordinate Drone::getLocation() { return location; }

QGeoCoordinate Drone::getIndivDestination() { return indivDestination; }

QGeoCoordinate Drone::getSwarmDestination() { return swarmDestination; }

QDateTime Drone::getLastModifiedTime() { return lastModified; }

/* Setter */

void Drone::setActivated(bool flag) { activated = flag; }

void Drone::setDroneNumber(int num) { droneNumber = num; }

void Drone::setStarted(bool flag) { started = flag; }

void Drone::setInSwarm(bool flag) { inSwarm = flag; }

// void Drone::setMoving(bool flag) { moving = flag; }

void Drone::setLocation(float lat, float lon) {
    location.setLatitude(lat);
    location.setLongitude(lon);
}

void Drone::setIndivDestination(float lat, float lon) {
    indivDestination.setLatitude(lat);
    indivDestination.setLongitude(lon);
}

void Drone::setSwarmDestination(float lat, float lon) {
    swarmDestination.setLatitude(lat);
    swarmDestination.setLongitude(lon);
}

void Drone::setLastModifiedTime(QDateTime newTime) { lastModified = newTime; }

/* Other Public */

bool Drone::isActivated() { return activated; }

bool Drone::isStarted() { return started; }

bool Drone::isInSwarm() { return inSwarm; }

// bool Drone::isMoving() { return moving; }
