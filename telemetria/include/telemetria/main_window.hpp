#ifndef telemetria_MAIN_WINDOW_H
#define telemetria_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "teleop_node.hpp"
#include "monit_node.hpp"

namespace telemetria
{

class Ponteiro : public QWidget
{
    Q_OBJECT

private:
    int ang;
    QColor cor;

public:
    Ponteiro(QWidget *parent = 0);
    void angulo(int novo_angulo);

protected:
    void paintEvent(QPaintEvent *event);
    void colorir(QColor nova_cor);
};

class MainWindow : public QMainWindow
{

  Q_OBJECT

public:

	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    void closeEvent(QCloseEvent *event);

public Q_SLOTS:

    void sair();
    void iniciar_autonav();
    void iniciar_teleop();
    void iniciar_joystick();
    void trocar_operacao();

    void on_dial_valueChanged(int valor);
    void on_slider_caracol_valueChanged(int valor);
    void on_slider_ang_valueChanged(int valor);
    void on_slider_lin_valueChanged(int valor);
    void on_slider_dir_valueChanged(int valor);
    void on_slider_esq_valueChanged(int valor);

    void on_campo_max_dir_textChanged(QString qstr);
    void on_campo_max_esq_textChanged(QString qstr);

    void on_up_ang_pressed();
    void on_up_lin_pressed();
    void on_up_dir_pressed();
    void on_up_esq_pressed();
    void on_up_base_pressed();
    void on_up_caracol_pressed();
    void on_down_ang_pressed();
    void on_down_lin_pressed();
    void on_down_dir_pressed();
    void on_down_esq_pressed();
    void on_down_base_pressed();
    void on_down_caracol_pressed();

    void on_campo_ang_textChanged(QString qstr);
    void on_campo_lin_textChanged(QString qstr);
    void on_campo_base_textChanged(QString qstr);
    void on_campo_caracol_textChanged(QString qstr);
    void on_campo_dir_textChanged(QString qstr);
    void on_campo_esq_textChanged(QString qstr);
    void on_vel_base_valueChanged(int valor);
    void on_vel_caracol_valueChanged(int valor);

    void base_dir();
    void caracol_dir();

    void on_bot_esq_clicked();
    void on_bot_dir_clicked();
    void on_bot_garra_clicked();
    void on_bot_panico_clicked();

    void mostra_prop();
    void mostra_base();
    void mostra_caracol();
    void mostra_garra();
    void mostra_imu();
    void mostra_ultrassom();
    void mostra_bateria();
    void mostra_camera();

private:

    QIntValidator *valid_base, *valid_caracol;
    QDoubleValidator *valid_dir, *valid_esq, *valid_ang, *valid_lin;
    Ponteiro *pont_base, *pont_imu, *pont_esq, *pont_dir;

    Ui::MainWindowDesign ui;
    TeleopNode teleop;
    MonitNode monit;
};

}  // namespace telemetria

#endif // telemetria_MAIN_WINDOW_H
