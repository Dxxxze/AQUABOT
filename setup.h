#ifndef SETUP_H
#define SETUP_H

#include <QDebug>
#include <QDoubleValidator>
#include <QDialog>
#include <QIntValidator>
#include <QGeoCoordinate>
#include "qpushbutton.h"

namespace Ui {
class Setup;
}

class Setup : public QDialog
{
    Q_OBJECT

public:
    explicit Setup(QWidget *parent = nullptr);
    ~Setup();
    /* getter */
    bool getSetupComplete();
    int getAvailableDrones();
    int getSensorFrequency();
    double getDroneDistance();
    int getXaxisRange();
    /* setter */
    void setSetupComplete(bool flag);
    /* Other */
    void checkEnable();
    void disableDronesSelection();
    bool isDroneSelectionDisabled();

private slots:
    void on_sensorFrequencyLineEdit_textChanged(const QString &arg1);
    void on_droneDistanceLineEdit_textChanged(const QString &arg1);
    void on_xRangeLineEdit_textChanged(const QString &arg1);
    void on_buttonBox_accepted();

private:
    Ui::Setup *ui;
    bool setupComplete;
    bool centerDisabled;
    bool initialized;

};

#endif // SETUP_H
