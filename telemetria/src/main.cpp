#include <QtGui>
#include <QApplication>
#include "../include/telemetria/main_window.hpp"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    telemetria::MainWindow w(argc,argv);
    w.resize(560,300);
    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    return app.exec();
}
