/* ********************************
 * FILE:        main.cpp
 * AUTHOR:      SIWEN WANG
 * COURSE:      SIE 498 2022 - 2023
 * PROJECT:     AQUABOT C3
 * ********************************/

#include "mainwindow.h"

#include <QHBoxLayout>

int main(int argc, char *argv[])
{
    /* below are failed attempt to work with different DPI screen
    I keep them for reference and future development */
    // QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    // QApplication::setAttribute(Qt::AA_DisableHighDpiScaling);
    // qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "1.5");

    argc = 3;
    argv[0] = (char*)"Appname";
    argv[1] = (char*)"--platform";
    argv[2] = (char*)"windows:dpiawareness=1";
    QApplication a(argc, argv);

    Setup setup;
    MainWindow w;
    // show the setup window
    setup.show();
    setup.exec();
    // show main window if ok is pressed
    if (setup.getSetupComplete()) {
        w.feedInitialSetup(&setup);
        w.show();
    }

    return a.exec();
}
