#include <QtGui>
#include <QApplication>
#include "main_window.hpp"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);   // Inicializa aplicação

    telemetria::MainWindow w(argc,argv);    // Inicializa janela
    w.resize(560,300);  // Redimensiona janela
    w.show();   // Exibe a janela

    // Faz com que fechar a janela encerre a aplicação
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    return app.exec();  // Roda o loop de eventos
}
