/* ********************************
 * FILE:        mainwindow.h
 * AUTHOR:      SIWEN WANG
 * COURSE:      SIE 498 2022 - 2023
 * PROJECT:     AQUABOT C3
 * ********************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "drone.h"
#include "setup.h"
#include <string>
#include <windows.h>
#include <QApplication>
#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QGeoCoordinate>
#include <QMainWindow>
#include <QMessageBox>
#include <QMetaObject>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQuickItem>
#include <QQuickView>
#include <QQuickWidget>
#include <QScrollArea>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void feedInitialSetup(Setup *setup);

private slots:
    void updateData();                          // timer
    void on_actionSetup_triggered();
    void on_selectCenter_currentTextChanged(const QString &arg1);
    void on_droneSize_valueChanged(int arg1);
    void on_plotDisplay_currentTextChanged(const QString &arg1);
    void on_autoUpdate_stateChanged(int arg1);
    void on_selectDestination_clicked(); 
    void on_go_clicked();
    void on_cancel_clicked();
    void on_start_clicked();
    void on_stop_clicked();
    void on_recallAll_clicked();

private:
    /* GUI Related */
    Ui::MainWindow *ui;
    QObject *mapObject;
    Setup *setup;

    /* File path */
    QString path; 
    QString currDir;                            // current directory that hold all the files
    
    /* Drone Control */
    const static int NUMBER_OF_DRONES = 8;
    Drone drones[NUMBER_OF_DRONES];
    bool selectingDestination;                  // selection enabled

    /* user defined variables */
    int plotDrone;                              // display this drone's data on the plots
    int sensorFrequency;                        // frequency of sensors/s
    double safeDistance;                        // distance between drones
    int xAxisRange;                             // range of plots x axis in seconds
    
    /* Timer */
    QTimer *timer;                              // timer to check data.txt
    QTime start;                                // start time of the program

    /* private helper functions */
    void initializeGUI();
    void updateSetup();
    void updateActivatedDrones(unsigned char allDrones);
    void sendSwarmCommand(QString command);
    void sendCommand(const char *str);
    bool eventFilter(QObject *obj, QEvent *event);
    void calculateSwarmDestination(QGeoCoordinate dest);
    void gridSwarmCommand(QList<int> swarm, QGeoCoordinate dest);
    void lineSwarmCommand(QList<int> swarm, QGeoCoordinate dest);

};
#endif // MAINWINDOW_H
