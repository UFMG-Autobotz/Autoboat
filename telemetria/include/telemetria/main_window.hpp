#ifndef telemetria_MAIN_WINDOW_HPP_
#define telemetria_MAIN_WINDOW_HPP_

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "teleop_node.hpp"
#include "monit_node.hpp"
#include "ponteiro.hpp"

namespace telemetria
{

class MainWindow : public QMainWindow
{

  Q_OBJECT

public:

	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

public Q_SLOTS:

    // Inicialização / Saída
    void sair();
    void iniciar_autonav();
    void iniciar_teleop();
    void iniciar_joystick();
    void trocar_operacao();

    // Quando os sliders (ou o dial) são arrastados
    void on_dial_valueChanged(int valor);
    void on_slider_caracol_valueChanged(int valor);
    void on_slider_ang_valueChanged(int valor);
    void on_slider_lin_valueChanged(int valor);
    void on_slider_dir_valueChanged(int valor);
    void on_slider_esq_valueChanged(int valor);

    // Funções que atualizam os limites dos sliders
    void on_campo_max_dir_textChanged(QString qstr);
    void on_campo_max_esq_textChanged(QString qstr);

    // Botões de incrementar e decrementar
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

    // Campos numéricos
    void on_campo_ang_textChanged(QString qstr);
    void on_campo_lin_textChanged(QString qstr);
    void on_campo_base_textChanged(QString qstr);
    void on_campo_caracol_textChanged(QString qstr);
    void on_campo_dir_textChanged(QString qstr);
    void on_campo_esq_textChanged(QString qstr);
    void on_vel_base_valueChanged(float valor);
    void on_vel_caracol_valueChanged(float valor);

    // Caixas de seleção de direção
    void base_dir();
    void caracol_dir();

    // Botões
    void on_bot_esq_clicked();
    void on_bot_dir_clicked();
    void on_bot_garra_clicked();
    void on_bot_panico_clicked();

    // LEDs
    void on_led_azul_clicked(bool estado);
    void on_led_laranja_clicked(bool estado);
    void on_led_verde_clicked(bool estado);

    // Mostradores da aba de monitoramento
    void mostra_prop();
    void mostra_base();
    void mostra_caracol();
    void mostra_garra();
    void mostra_imu();
    void mostra_ultrassom();
    void mostra_bateria();
    void mostra_camera();

    // Funções especiais para a propulsão
    void muda_esq_dir(float esq, float dir, bool continua);
    void muda_lin_ang(float lin, float ang, bool continua);
    void atualiza_posicao();

private:

    QIntValidator *valid_base, *valid_caracol;
    QDoubleValidator *valid_dir, *valid_esq, *valid_ang, *valid_lin;
    Ponteiro *pont_base, *pont_imu, *pont_esq, *pont_dir;
    float vel_ang, vel_lin;

    Ui::MainWindowDesign ui;  // Objeto que armazena a GUI desenhada
    TeleopNode teleop;  // Nó de teleoperação
    MonitNode monit;  // Nó de monitoramento
};

}  // namespace telemetria

#endif // telemetria_MAIN_WINDOW_HPP_
