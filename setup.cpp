#include "setup.h"
#include "ui_setup.h"

Setup::Setup(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Setup)
{
    ui->setupUi(this);
    setupComplete = false;
    centerDisabled = false;
    initialized = false;
    // validators for line edits
    ui -> sensorFrequencyLineEdit -> setValidator(new QIntValidator(1, 2147483647, this));
    ui -> droneDistanceLineEdit -> setValidator(new QDoubleValidator(7, 2147483647, 2, this));
    ui -> xRangeLineEdit -> setValidator(new QIntValidator(this));
}

Setup::~Setup() { delete ui; }

/* ----------------------------------------------------------Public Methods */

/* Getter */

bool Setup::getSetupComplete() { return setupComplete; }

/*
 * Use binary to tell which drones are available
 * Drone 1 to 8 from MSB to LSB
 */
int Setup::getAvailableDrones() {
    unsigned char drones = 0;
    if (ui -> drone1 -> checkState() == Qt::Checked)
        drones = drones | 128;
    if (ui -> drone2 -> checkState() == Qt::Checked)
        drones = drones | 64;
    if (ui -> drone3 -> checkState() == Qt::Checked)
        drones = drones | 32;
    if (ui -> drone4 -> checkState() == Qt::Checked)
        drones = drones | 16;
    if (ui -> drone5 -> checkState() == Qt::Checked)
        drones = drones | 8;
    if (ui -> drone6 -> checkState() == Qt::Checked)
        drones = drones | 4;
    if (ui -> drone7 -> checkState() == Qt::Checked)
        drones = drones | 2;
    if (ui -> drone8 -> checkState() == Qt::Checked)
        drones = drones | 1;
    return drones;
}

int Setup::getSensorFrequency() { return ui -> sensorFrequencyLineEdit -> text().toInt(); }

double Setup::getDroneDistance() { return ui -> droneDistanceLineEdit -> text().toDouble(); }

int Setup::getXaxisRange() { return ui -> xRangeLineEdit -> text().toInt(); }

/* Setter */

void Setup::setSetupComplete(bool flag) { setupComplete = flag; }

/* Check whether it's okay to enable ok button now */
void Setup::checkEnable() {
    if (!ui->sensorFrequencyLineEdit->text().isEmpty()
            && !ui->droneDistanceLineEdit->text().isEmpty()
            && !ui->xRangeLineEdit->text().isEmpty())
    ui -> buttonBox -> button(QDialogButtonBox::Ok) -> setEnabled(true);
}

void Setup::disableDronesSelection() {
    ui -> label1 -> setEnabled(false);
    ui -> drone1 -> setEnabled(false);
    ui -> drone2 -> setEnabled(false);
    ui -> drone3 -> setEnabled(false);
    ui -> drone4 -> setEnabled(false);
    ui -> drone5 -> setEnabled(false);
    ui -> drone6 -> setEnabled(false);
    ui -> drone7 -> setEnabled(false);
    ui -> drone8 -> setEnabled(false);
    initialized = true;
}

bool Setup::isDroneSelectionDisabled() { return initialized; }

/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------- Slots */

void Setup::on_sensorFrequencyLineEdit_textChanged(const QString &arg1) {
    if (arg1.length() == 0) ui -> buttonBox -> button(QDialogButtonBox::Ok) -> setEnabled(false);
    else checkEnable();
}


void Setup::on_droneDistanceLineEdit_textChanged(const QString &arg1) {
    if (arg1.length() == 0) ui -> buttonBox -> button(QDialogButtonBox::Ok) -> setEnabled(false);
    else checkEnable();
}


void Setup::on_xRangeLineEdit_textChanged(const QString &arg1) {
    if (arg1.length() == 0) ui -> buttonBox -> button(QDialogButtonBox::Ok) -> setEnabled(false);
    else checkEnable();
}

void Setup::on_buttonBox_accepted() { setupComplete = true; }

/* ------------------------------------------------------------------------- */





