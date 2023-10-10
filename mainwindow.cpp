/* ********************************
 * FILE:        mainwindow.cpp
 * AUTHOR:      SIWEN WANG
 * COURSE:      SIE 498 2022 - 2023
 * PROJECT:     AQUABOT C3
 * ********************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui -> setupUi(this);
    /* User defined variables */
    plotDrone = 1;
    safeDistance = 7;
    xAxisRange = 20;

    /* initialize window and initialize the plots, table and map */
    initializeGUI();
    /* initialize Drone objects and drone control variables */
    path = qApp -> applicationDirPath();
    qInfo() << path;
    
    currDir = "C:\\Users\\Draco\\OneDrive\\AQUABOT";
    for (int i = 0; i < NUMBER_OF_DRONES; i++) {
        Drone temp;
        temp.setDroneNumber(i + 1);
        QFileInfo tempFile(currDir + "\\drone" + QString::number(i + 1) + "\\data.txt");
        temp.setLastModifiedTime(tempFile.lastModified());
        drones[i] = temp;
    }
    selectingDestination = false;

    /* start timer */
    timer = new QTimer(this);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(updateData()));
    timer -> start(10);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::feedInitialSetup(Setup *set) {
    setup = set;
    setup -> setSetupComplete(false);
    updateSetup();
    setup -> disableDronesSelection();
}

/* ----------------------------------------------------------Private Methods */

/*
 * Initialize the data table, plots and qml map
 */
void MainWindow::initializeGUI() {
    // initialize the table
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 5; c++) {
            QTableWidgetItem *item = new QTableWidgetItem;
            item -> setText("-");
            item -> setTextAlignment(Qt::AlignCenter);
            ui -> tableWidget -> setItem(r, c, item);
        }
    }
    ui -> tableWidget -> setColumnWidth(3, 300);
    // initialize the plot
    ui -> pHPlot -> addGraph();
    ui -> tempPlot -> addGraph();
    ui -> salinityPlot -> addGraph();
    // configure timer ticker for x axis
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker -> setTimeFormat("%h:%m:%s");
    ui -> pHPlot -> xAxis -> setTicker(timeTicker);
    ui -> tempPlot -> xAxis -> setTicker(timeTicker);
    ui -> salinityPlot -> xAxis -> setTicker(timeTicker);
    // set up axes
    ui -> pHPlot -> xAxis -> setLabel("Time (MST)");
    ui -> tempPlot -> xAxis -> setLabel("Time (MST)");
    ui -> salinityPlot -> xAxis -> setLabel("Time (MST)");
    start = QTime().currentTime();
    int currentTime = start.hour() * 60 * 60 + start.minute() * 60 + start.second();
    ui -> pHPlot -> xAxis -> setRange(currentTime, currentTime + xAxisRange);
    ui -> tempPlot -> xAxis -> setRange(currentTime, currentTime + xAxisRange);
    ui -> salinityPlot -> xAxis -> setRange(currentTime, currentTime + xAxisRange);
    ui -> pHPlot -> yAxis -> setLabel("pH");
    ui -> pHPlot -> yAxis -> setRange(0, 14);
    ui -> tempPlot -> yAxis -> setLabel("Temperature");
    ui -> tempPlot -> yAxis -> setRange(-5, 50);
    ui -> salinityPlot -> yAxis -> setLabel("Salinity");
    ui -> salinityPlot -> yAxis -> setRange(0, 1000);
    // set up map
    ui -> map -> setSource(QUrl(QStringLiteral("qrc:/map.qml")));
    ui -> map -> show();
    mapObject = ui -> map -> rootObject();
    ui -> autoUpdate -> setCheckState(Qt::Checked);
    // mouse change
    QCoreApplication::instance()->installEventFilter(this);
}

/* Update attributes based on user input */
void MainWindow::updateSetup() {
    // update available drones
    if (!setup -> isDroneSelectionDisabled())
        updateActivatedDrones(setup -> getAvailableDrones());

    // update sensor frequency
    sensorFrequency = setup -> getSensorFrequency();
    QString command = "SensorFrequency " + QString::number(sensorFrequency);
    // update drone distance
    safeDistance = setup -> getDroneDistance();
    // TODO: can recalculate if there's a ongoing swarm command
    // update xaxis ranges on plot
    int newRange = setup -> getXaxisRange();
    if (xAxisRange == newRange) return;
    xAxisRange = newRange;
    QTime ct = QTime().currentTime();
    int currentTime = ct.hour() * 60 * 60 + ct.minute() * 60 + ct.second();
    if (start.secsTo(ct) > xAxisRange) {
        ui -> pHPlot -> xAxis -> setRange(currentTime - xAxisRange, currentTime);
        ui -> tempPlot -> xAxis -> setRange(currentTime - xAxisRange, currentTime);
        ui -> salinityPlot -> xAxis -> setRange(currentTime - xAxisRange, currentTime);
    } else {
        ui -> pHPlot -> xAxis -> setRange(currentTime, currentTime + xAxisRange);
        ui -> tempPlot -> xAxis -> setRange(currentTime, currentTime + xAxisRange);
        ui -> salinityPlot -> xAxis -> setRange(currentTime, currentTime + xAxisRange);
    }
    ui -> pHPlot -> graph(0) -> rescaleValueAxis();
    ui -> tempPlot -> graph(0) -> rescaleValueAxis();
    ui -> salinityPlot -> graph(0) -> rescaleValueAxis();
    ui -> pHPlot -> replot();
    ui -> tempPlot -> replot();
    ui -> salinityPlot -> replot();
}

/* Use binary to represent which drones are available */
void MainWindow::updateActivatedDrones(unsigned char allDrones) {
    if ((allDrones & 1) > 0) drones[7].setActivated(true);
    else {
        ui ->plotDisplay->removeItem(7);
        ui -> selectStartStop -> removeItem(8);
        ui -> selectCenter -> removeItem(7);
        ui -> select -> removeItem(8);
    }
    if ((allDrones & 2) > 0) drones[6].setActivated(true);
    else {
        ui ->plotDisplay->removeItem(6);
        ui -> selectStartStop -> removeItem(7);
        ui -> selectCenter -> removeItem(6);
        ui -> select -> removeItem(7);
    }
    if ((allDrones & 4) > 0) drones[5].setActivated(true);
    else {
        ui ->plotDisplay->removeItem(5);
        ui -> selectStartStop -> removeItem(6);
        ui -> selectCenter -> removeItem(5);
        ui -> select -> removeItem(6);
    }
    if ((allDrones & 8) > 0) drones[4].setActivated(true);
    else {
        ui ->plotDisplay->removeItem(4);
        ui -> selectStartStop -> removeItem(5);
        ui -> selectCenter -> removeItem(4);
        ui -> select -> removeItem(5);
    }
    if ((allDrones & 16) > 0) drones[3].setActivated(true);
    else {
        ui ->plotDisplay->removeItem(3);
        ui -> selectStartStop -> removeItem(4);
        ui -> selectCenter -> removeItem(3);
        ui -> select -> removeItem(4);
    }
    if ((allDrones & 32) > 0) drones[2].setActivated(true);
    else {
        ui ->plotDisplay->removeItem(2);
        ui -> selectStartStop -> removeItem(3);
        ui -> selectCenter -> removeItem(2);
        ui -> select -> removeItem(3);
    }
    if ((allDrones & 64) > 0) drones[1].setActivated(true);
    else {
        ui ->plotDisplay->removeItem(1);
        ui -> selectStartStop -> removeItem(2);
        ui -> selectCenter -> removeItem(1);
        ui -> select -> removeItem(2);
    }
    if ((allDrones & 128) > 0) drones[0].setActivated(true);
    else {
        ui ->plotDisplay->removeItem(0);
        ui -> selectStartStop -> removeItem(1);
        ui -> selectCenter -> removeItem(0);
        ui -> select -> removeItem(1);
    }
    if (allDrones == 0) {
        ui -> selectStartStop -> setEnabled(false);
        ui -> selectCenter -> setEnabled(false);
        ui -> select -> setEnabled(false);
        ui -> recallAll -> setEnabled(false);
        ui -> go -> setEnabled(false);
        ui -> cancel -> setEnabled(false);
        ui -> start -> setEnabled(false);
        ui -> stop -> setEnabled(false);
    }
}

/* Send the command to all drones in swarm currently */
void MainWindow::sendSwarmCommand(QString command) {
    for (int i = 0; i < NUMBER_OF_DRONES; i++) {
        if (drones[i].isInSwarm() && drones[i].isActivated())
            command += " " + QString::number(i + 1);
    }
    sendCommand(command.toLocal8Bit().data());
}

/*
 * Parameters:      str, a String of command
 * Return:          N/A
 * Purpose:         Write the input command into command.txt
 */
void MainWindow::sendCommand(const char *str) {
    // qInfo() << "Sending command";
    QFile command("C:\\Users\\Draco\\OneDrive\\AQUABOT\\command.txt");
    if (!command.open(QIODevice::WriteOnly)) {
        qCritical() << "Fail to open file";
        qCritical() << command.errorString();
        return;
    }
    command.write(str);
    command.flush();
    command.close();
}

/* Change cursor */
bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
    if (obj ==  ui -> map) {
        QEvent::Type type = event->type();
        if  (type == QEvent::Leave && selectingDestination)
            setCursor(Qt::ArrowCursor);
        else if (type == QEvent::Enter && selectingDestination)
            setCursor(Qt::CrossCursor);
    }
    return QWidget::eventFilter(obj, event);
}

/* Calculate destination of each drones in the swarm */
void MainWindow::calculateSwarmDestination(QGeoCoordinate dest) {
    // ordered list of all drones in swarm
    QList<int> swarm;
    for (int i = 0; i < NUMBER_OF_DRONES; i++) {
        if (drones[i].isActivated() && drones[i].isInSwarm()) {
            swarm.append(i + 1);
            drones[i].setStarted(true);
        } else drones[i].setInSwarm(false);
    }

    // if none, error
    if (swarm.size() == 0) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","No drone in swarm at this time. ");
        messageBox.setFixedSize(600, 200);
        return;
    }

    // send gps destination based on swarm pattern
    if (ui->selectPattern->currentText() == "Grid")
        gridSwarmCommand(swarm, dest);
    else if (ui->selectPattern->currentText() == "Line")
        lineSwarmCommand(swarm, dest);
}

void MainWindow::gridSwarmCommand(QList<int> swarm, QGeoCoordinate dest) {
    QString command = "";
    QVariant arg1;
    QVariant arg2;
    QVariant arg3;

    for (int i = 0; i < swarm.size(); i++) {
        QGeoCoordinate temp;
        if (i == 0)
            temp = dest;
        else if (i == 1)
            temp = dest.atDistanceAndAzimuth(safeDistance, 90, 0);
        else if (i == 2)
            temp = dest.atDistanceAndAzimuth(safeDistance * 2, 90, 0);
        else if (i == 3)
            temp = dest.atDistanceAndAzimuth(safeDistance, 180, 0);
        else if (i == 4)
            temp = dest.atDistanceAndAzimuth(qSqrt(5 * safeDistance * safeDistance), qAtan(0.5) * (180.0/3.141592653589793238463) + 90, 0);
        else if (i == 5)
            temp = dest.atDistanceAndAzimuth(safeDistance * 2, 180, 0);
        else if (i == 6)
            temp = dest.atDistanceAndAzimuth(qSqrt(5 * safeDistance * safeDistance), 180 - qAtan(0.5) * (180.0/3.141592653589793238463), 0);
        else if (i == 7)
            temp = dest.atDistanceAndAzimuth(safeDistance * qSqrt(2), 135, 0);

        arg1 = swarm.at(i);
        arg2 = temp.latitude();
        arg3 = temp.longitude();
        QMetaObject::invokeMethod(mapObject, "setDest", Q_ARG(QVariant, arg1), Q_ARG(QVariant, arg2), Q_ARG(QVariant, arg3));
        // update drone object
        drones[swarm.at(i) - 1].setSwarmDestination(temp.latitude(), temp.longitude());
        drones[swarm.at(i) - 1].setIndivDestination(360, 360);
        // send command
        command += QString::number(swarm.at(i));
        command += " ";
        command += QString::number(temp.latitude(), 'f', 6);
        command += ",";
        command += QString::number(temp.longitude(), 'f', 6);
        command += " ";
    }

    sendCommand(command.toLocal8Bit().data());
}

void MainWindow::lineSwarmCommand(QList<int> swarm, QGeoCoordinate dest) {
    QString command = "";
    int center = swarm.size() / 2;          // center drone index

    QVariant arg1;
    QVariant arg2;
    QVariant arg3;

    for (int i = 0; i < swarm.size(); i++) {
        qreal azimuth;
        if (i == center) {
            azimuth = 0;
        } else if (i < center) azimuth = -90;
        else azimuth = 90;

        // calculate gps coordinate
        qreal distance = safeDistance * qAbs(center - i);
        QGeoCoordinate temp = dest.atDistanceAndAzimuth(distance, azimuth, 0);
        // update dest pin on map
        arg1 = swarm.at(i);
        arg2 = temp.latitude();
        arg3 = temp.longitude();
        QMetaObject::invokeMethod(mapObject, "setDest", Q_ARG(QVariant, arg1), Q_ARG(QVariant, arg2), Q_ARG(QVariant, arg3));

        // update drone object
        drones[swarm.at(i) - 1].setSwarmDestination(temp.latitude(), temp.longitude());
        drones[swarm.at(i) - 1].setIndivDestination(360, 360);
        // send command
        command += QString::number(swarm.at(i));
        command += " ";
        command += QString::number(temp.latitude(), 'f', 6);
        command += ",";
        command += QString::number(temp.longitude(), 'f', 6);
        command += " ";
    }

    sendCommand(command.toLocal8Bit().data());
}

/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------- Slots */

/* Update new data */
void MainWindow::updateData() {
    // update x axis
    QTime ct = QTime().currentTime();
    int currentTime = ct.hour() * 60 * 60 + ct.minute() * 60 + ct.second();
    if (start.secsTo(ct) > xAxisRange) {
        // reset the range when the plot should move
        ui -> pHPlot -> xAxis -> setRange(currentTime - xAxisRange, currentTime);
        ui -> tempPlot -> xAxis -> setRange(currentTime - xAxisRange, currentTime);
        ui -> salinityPlot -> xAxis -> setRange(currentTime - xAxisRange, currentTime);
    }
    ui -> pHPlot -> graph(0) -> rescaleValueAxis();
    ui -> tempPlot -> graph(0) -> rescaleValueAxis();
    ui -> salinityPlot -> graph(0) -> rescaleValueAxis();

    // process the new data from data file of each drones
    // data from drones will be in this format: 
    // GMT YYYY/mm/dd HH:MM:SS N pH temp sal lat,lon bearing
    for (int i = 0; i < NUMBER_OF_DRONES; i++) {
        if (!drones[i].isActivated()) continue;
        QString droneNum = QString::number(i + 1);
        QFileInfo tempFile("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + droneNum + "\\data.txt");
        if (tempFile.lastModified().secsTo(drones[i].getLastModifiedTime()) != 0) {
            drones[i].setLastModifiedTime(tempFile.lastModified());

            // open data file
            QFile file("C:\\Users\\Draco\\OneDrive\\AQUABOT\\drone" + droneNum + "\\data.txt");
            // if failed to open file
            if (!file.open(QIODevice::ReadOnly)) {
                qCritical() << "Fail to open drone" << droneNum << "data file";
                qCritical() << file.errorString();
                continue;
            }

            // read new data message
            QString line = file.readLine();
            file.close();
            QStringList datas = line.split(" ");
            if(datas.size() < 9) continue;

            // get the time to display
            // converting GMT to MST
            int newTime = 0;
            if (i + 1 == plotDrone) {
                QStringList timeData = datas.at(2).split(":");
                newTime = timeData.at(0).toInt() * 3600
                            + timeData.at(1).toInt() * 60
                            + timeData.at(2).toInt();
                //            - 7 * 60 * 60;
                // if (newTime < 0) newTime += 24 * 60 * 60;
            }
            // process data
            datas.removeFirst();    // GMT
            datas.removeFirst();    // YYYY/mm/dd
            datas.removeFirst();    // HH:MM:SS
            datas.removeFirst();    // N
            // pH data
            if (QString::compare(datas.at(0), "-", Qt::CaseSensitive) != 0) {
                ui -> tableWidget -> item(i, 0) -> setText(datas.at(0));
                if (i + 1 == plotDrone)
                    ui -> pHPlot -> graph(0) -> addData(newTime, datas.at(0).toDouble());
            }
            datas.removeFirst();    // pH
            // temperature data
            if (QString::compare(datas.at(0), "-", Qt::CaseSensitive) != 0) {
                ui -> tableWidget -> item(i, 1) -> setText(datas.at(0));
                if (i + 1 == plotDrone)
                    ui -> tempPlot -> graph(0) -> addData(newTime, datas.at(0).toDouble());
            }
            datas.removeFirst();    // temp
            // salinity data
            if (QString::compare(datas.at(0), "-", Qt::CaseSensitive) != 0) {
                ui -> tableWidget -> item(i, 2) -> setText(datas.at(0));
                if (i + 1 == plotDrone)
                    ui -> salinityPlot -> graph(0) -> addData(newTime, datas.at(0).toDouble());
            }
            datas.removeFirst();    // sal
            // GPS
            QStringList gps = datas.at(0).split(",");
            double lat = gps.at(0).toDouble();
            double lon = gps.at(1).toDouble();
            drones[i].setLocation(lat, lon);
            ui -> tableWidget -> item(i, 3) -> setText(datas.at(0));
            datas.removeFirst();    // lat,lon
            // bearing
            double bearing = datas.at(0).toDouble();
            ui -> tableWidget -> item(i, 4) -> setText(datas.at(0));
            // update on map
            QMetaObject::invokeMethod(mapObject, "updateDrone",
                                      Q_ARG(int, i + 1),
                                      Q_ARG(double, lat),
                                      Q_ARG(double, lon),
                                      Q_ARG(double, bearing),
                                      Q_ARG(bool, drones[i].isInSwarm()));
        }
    }

    ui -> pHPlot -> replot();
    ui -> tempPlot -> replot();
    ui -> salinityPlot -> replot();
}

/* Handle when the edit setup menu clicked */
void MainWindow::on_actionSetup_triggered() {
    // show the setup window
    setup -> show();
    setup -> exec();
    // show main window if ok is pressed
    if (setup -> getSetupComplete()) {
        updateSetup();
        setup ->setSetupComplete(false);
    }
}

/* Change the display center when auto updating on map */
void MainWindow::on_selectCenter_currentTextChanged(const QString &arg1) {
    QVariant arg = arg1.at(6);
    QMetaObject::invokeMethod(mapObject, "changeCenter", Q_ARG(QVariant, arg));
}

/* Change the size of the drone item on map */
void MainWindow::on_droneSize_valueChanged(int arg1) {
    QVariant arg = arg1;
    QMetaObject::invokeMethod(mapObject, "updateDroneSize", Q_ARG(QVariant, arg));
}

/* Change the data on the plots */
void MainWindow::on_plotDisplay_currentTextChanged(const QString &arg1) {
    plotDrone = arg1[6].digitValue();
    // remove data from all plots
    ui->pHPlot->graph(0)->data()->clear();
    ui->tempPlot->graph(0)->data()->clear();
    ui->salinityPlot->graph(0)->data()->clear();
    // replot
    ui -> pHPlot -> replot();
    ui -> tempPlot -> replot();
    ui -> salinityPlot -> replot();
}

/* Set Auto Update */
void MainWindow::on_autoUpdate_stateChanged(int arg1) {
    QVariant arg = arg1;
    QMetaObject::invokeMethod(mapObject, "setAutoUpdate", Q_ARG(QVariant, arg));
}

/* Select destination button clicked */
void MainWindow::on_selectDestination_clicked() {
    QPushButton *button = ui -> selectDestination;
    if (selectingDestination) {
        selectingDestination = false;
        QMetaObject::invokeMethod(mapObject, "setSelectingDestination",
                                  Q_ARG(bool, selectingDestination));
        button -> setStyleSheet("QPushButton {background-color: white; "
                                "color: black; font: 12pt \"Calibri\";}");
        QMetaObject::invokeMethod(mapObject, "removePin");
    } else {
        selectingDestination = true;
        button -> setStyleSheet("QPushButton {background-color: #aa2116; "
                                "color: #FFFFFF; font: 12pt \"Calibri\";}");
        QMetaObject::invokeMethod(mapObject, "setSelectingDestination",
                                  Q_ARG(bool, selectingDestination));
    }
}

/* GO button clicked */
void MainWindow::on_go_clicked() {
    // check error
    QObject *pin = qvariant_cast<QObject *>(mapObject -> property("pin"));
    if (pin == NULL) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","No destination selected yet!");
        messageBox.setFixedSize(600, 200);
        return;
    }
    // send command
    QGeoCoordinate dest = qvariant_cast<QGeoCoordinate>(pin->property("coordinate"));
    QString lat = QString::number(dest.latitude(), 'f', 6);
    QString lon = QString::number(dest.longitude(), 'f', 6);
    QString selected = ui -> select -> currentText();
    // if Swarm command
    if (selected == "Swarm") calculateSwarmDestination(dest);
    // if individual command
    else {
        // send command
        QString command = selected[6] + " " + lat +  "," + lon;
        sendCommand(command.toLocal8Bit().data());
        // add dest pin on map
        int droneNum = selected[6].digitValue();
        QVariant arg1 = droneNum;
        QVariant arg2 = dest.latitude();
        QVariant arg3 = dest.longitude();
        QMetaObject::invokeMethod(mapObject, "setDest", Q_ARG(QVariant, arg1), Q_ARG(QVariant, arg2), Q_ARG(QVariant, arg3));
        // update drone object
        drones[droneNum - 1].setInSwarm(false);
        drones[droneNum - 1].setIndivDestination(dest.latitude(), dest.longitude());
        drones[droneNum - 1].setSwarmDestination(360, 360); // invalid coordinate
    }
    // change cursor
    setCursor(Qt::ArrowCursor);
    selectingDestination = false;
    QPushButton *button = ui -> selectDestination;
    button -> setStyleSheet("QPushButton {background-color: white; "
                            "color: black; font: 12pt \"Calibri\";}");
    QMetaObject::invokeMethod(mapObject, "setSelectingDestination",
                              Q_ARG(bool, selectingDestination));
    QMetaObject::invokeMethod(mapObject, "removePin");
}

/* CANCEL button clicked */
void MainWindow::on_cancel_clicked() {
    QVariant arg1;
    QString selected = ui -> select -> currentText();
    // if swarm cancel
    if (selected == "Swarm") {
        // send command
        sendSwarmCommand("CANCEL");
        for (int i = 0; i < NUMBER_OF_DRONES; i++) {
            if (drones[i].isActivated() && drones[i].isInSwarm()) {
                // remove all pin from map
                arg1 = i + 1;
                QMetaObject::invokeMethod(mapObject, "removeDest", Q_ARG(QVariant, arg1));
                // update drone object
                drones[i].setSwarmDestination(360, 360);
            }
        }
    // if individual cancel
    } else {
        // send command
        QString command = "CANCEL " + selected[6];
        int droneNum = selected[6].digitValue();
        sendCommand(command.toLocal8Bit().data());
        // remove pin from map
        arg1 = droneNum;
        QMetaObject::invokeMethod(mapObject, "removeDest", Q_ARG(QVariant, arg1));

        // update drone object
        drones[droneNum - 1].setIndivDestination(360, 360);
    } 

    // change cursor
    setCursor(Qt::ArrowCursor);
    selectingDestination = false;
    QPushButton *button = ui -> selectDestination;
    button -> setStyleSheet("QPushButton {background-color: white; "
                            "color: black; font: 12pt \"Calibri\";}");
    QMetaObject::invokeMethod(mapObject, "setSelectingDestination",
                              Q_ARG(bool, selectingDestination));
    QMetaObject::invokeMethod(mapObject, "removePin");
}

/* START button clicked */
void MainWindow::on_start_clicked() {
    QString selected = ui -> selectStartStop -> currentText();
    // if swarm start
    if (selected == "Swarm") {
        // send command
        sendSwarmCommand("START");
        // update drone objects
        for (int i = 0; i < NUMBER_OF_DRONES; i++){
            if (drones[i].isActivated() && drones[i].isInSwarm())
                drones[i].setStarted(true);
            else if (!drones[i].isActivated()) drones[i].setInSwarm(false);
        }
    // if individual start
    } else {
        // send command
        QString command = "START " + selected[6];
        sendCommand(command.toLocal8Bit().data());
        // update drone object
        int droneNum = selected[6].digitValue();
        drones[droneNum - 1].setStarted(true);
    }
}

/* STOP button clicked */
void MainWindow::on_stop_clicked() {
    QString selected = ui -> selectStartStop -> currentText();
    // if swarm stop
    if (selected == "Swarm") {
        // send command
        sendSwarmCommand("STOP");
        // update drone object
        for (int i = 0; i < NUMBER_OF_DRONES; i++) {
            if (drones[i].isInSwarm() && drones[i].isActivated()) 
                drones[i].setStarted(false);
        }
    // if individual stop
    } else {
        // send command
        QString command = "STOP " + selected[6];
        sendCommand(command.toLocal8Bit().data());
        // update drone object
        int droneNum = selected[6].digitValue();
        drones[droneNum - 1].setStarted(false);
    }
}

void MainWindow::on_recallAll_clicked() {
    for (int i = 0; i < NUMBER_OF_DRONES; i++)
        if (drones[i].isActivated()) drones[i].setInSwarm(true);
    ui -> select -> setCurrentIndex(0);
    ui -> go -> click();
}

/* ------------------------------------------------------------------------- */



