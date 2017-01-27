#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/telemetria/main_window.hpp"

namespace telemetria
{

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , teleop(argc,argv)
    , monit(argc,argv)
//    , monit(argc, argv)
{
    ui.setupUi(this); // Conectar os sinais dos objetos da UI aos callbacks da forma on_...();

    setWindowIcon(QIcon(":/images/icon.png"));

    while(!monit.init())
    {
        QMessageBox msgBox;
        msgBox.setText("Nao foi possível iniciar o no de telemetria.\nVerifique se o rosmaster foi inicializado.");
        msgBox.exec();

        return;
    }

    ui.abas->removeTab(2);
    ui.abas->removeTab(1);

    QPixmap img_auto(":/images/auto.png");
    img_auto = img_auto.scaled(QSize(100,100));

    QLabel* label_auto = new QLabel(ui.aba_home);
    label_auto->setPixmap(img_auto);
    ui.layout_home->addWidget(label_auto,6,0,Qt::AlignCenter);

    QObject::connect(&teleop, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(ui.bot_nevermind, SIGNAL(clicked()), this, SLOT(sair()));
    QObject::connect(ui.bot_autonomo, SIGNAL(clicked()), this, SLOT(iniciar_autonav()));
    QObject::connect(ui.bot_joystick, SIGNAL(clicked()), this, SLOT(iniciar_joystick()));
    QObject::connect(ui.bot_teleop, SIGNAL(clicked()), this, SLOT(iniciar_teleop()));
    QObject::connect(ui.bot_trocar, SIGNAL(clicked()), this, SLOT(trocar_operacao()));

    QObject::connect(&monit, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&monit, SIGNAL(atual_prop()), this, SLOT(mostra_prop()));
    QObject::connect(&monit, SIGNAL(atual_base()), this, SLOT(mostra_base()));
    QObject::connect(&monit, SIGNAL(atual_caracol()), this, SLOT(mostra_caracol()));
    QObject::connect(&monit, SIGNAL(atual_garra()), this, SLOT(mostra_garra()));
    QObject::connect(&monit, SIGNAL(atual_imu()), this, SLOT(mostra_imu()));
    QObject::connect(&monit, SIGNAL(atual_bateria()), this, SLOT(mostra_bateria()));
    QObject::connect(&monit, SIGNAL(atual_ultrassom()), this, SLOT(mostra_ultrassom()));
    QObject::connect(&monit, SIGNAL(atual_camera()), this, SLOT(mostra_camera()));

    pont_esq = new Ponteiro(ui.frame_prop);
    pont_dir = new Ponteiro(ui.frame_prop);
    pont_imu = new Ponteiro(ui.frame_imu);
    pont_base = new Ponteiro(ui.frame_base);

    ui.layout_base->addWidget(pont_base,1,0,1,2);
    ui.layout_imu->addWidget(pont_imu,2,1);
    ui.layout_prop->addWidget(pont_esq,4,1);
    ui.layout_prop->addWidget(pont_dir,4,2);

    ui.painel_base->raise();
    ui.painel_imu->raise();
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

void MainWindow::sair()
{
    close();
}

void MainWindow::iniciar_autonav()
{
//    ROS_INFO("Iniciando no de navegação autonoma...");

//    int i = system("rosrun turtlesim turtlesim_node");

//    ROS_INFO("%i",i);

    setUpdatesEnabled(false);

    ui.abas->clear();
    ui.abas->addTab(ui.aba_telemetr, tr("Monitoramento"));

    if(!isFullScreen() && !isMaximized())
        resize(840,500);

    setUpdatesEnabled(true);
}

void MainWindow::iniciar_joystick()
{
//    ROS_INFO("Iniciando no de controle via joystick");
//    int i = system("roslaunch autoboat_joystick autoboat_joystick.launch");
//    ROS_INFO("%i",i);

    setUpdatesEnabled(false);

    ui.abas->clear();
    ui.abas->addTab(ui.aba_telemetr, tr("Monitoramento"));

    if(!isFullScreen() && !isMaximized())
        resize(840,500);

    setUpdatesEnabled(true);
}

void MainWindow::iniciar_teleop()
{
    if(!teleop.init())
    {
        QMessageBox msgBox;
        msgBox.setText("Nao foi possível iniciar o no de teleop.\nVerifique se o rosmaster foi inicializado.");
        msgBox.exec();

        return;
    }

    ROS_INFO("Iniciando no de teleop");

    setUpdatesEnabled(false);

    ui.abas->clear();
    ui.abas->addTab(ui.aba_teleop, tr("Controle"));
    ui.abas->addTab(ui.aba_telemetr, tr("Monitoramento"));
    ui.abas->setCurrentIndex(0);

    valid_base = new QIntValidator(0,48,ui.campo_base);
    valid_caracol = new QIntValidator(0,100,ui.campo_caracol);
    valid_dir = new QDoubleValidator(0,50,1,ui.campo_dir);
    valid_esq = new QDoubleValidator(0,50,1,ui.campo_esq);
    valid_ang = new QDoubleValidator(-.625,.625,3,ui.campo_ang);
    valid_lin = new QDoubleValidator(-50,50,1,ui.campo_lin);

    ui.campo_base->setValidator(valid_base);
    ui.campo_caracol->setValidator(valid_caracol);
    ui.campo_ang->setValidator(valid_ang);
    ui.campo_lin->setValidator(valid_lin);
    ui.campo_dir->setValidator(valid_dir);
    ui.campo_esq->setValidator(valid_esq);
    ui.campo_max_dir->setValidator(new QIntValidator(0,1e6,ui.campo_max_dir));
    ui.campo_max_esq->setValidator(new QIntValidator(0,1e6,ui.campo_max_esq));

    QObject::connect(ui.bot_sair, SIGNAL(clicked()), this, SLOT(sair()));
    QObject::connect(ui.opt_base_esq, SIGNAL(toggled(bool)), this, SLOT(base_dir()));
    QObject::connect(ui.opt_base_dir, SIGNAL(toggled(bool)), this, SLOT(base_dir()));
    QObject::connect(ui.opt_caracol_esq, SIGNAL(toggled(bool)), this, SLOT(caracol_dir()));
    QObject::connect(ui.opt_caracol_dir, SIGNAL(toggled(bool)), this, SLOT(caracol_dir()));
    
    if(!isFullScreen() && !isMaximized())
        resize(840,500);

    setUpdatesEnabled(true);
}

void MainWindow::trocar_operacao()
{
    setUpdatesEnabled(false);

    ui.texto_boasvindas->setText(tr("Vejo que mudou de ideia, Autoboticista.\nDeseja trocar o modo de operação ou sair?"));
    ui.bot_nevermind->setFont(ui.bot_teleop->font());
    ui.bot_nevermind->setText(tr("Quero ir embora. Obrigado."));
    ui.layout_home->addWidget(ui.bot_nevermind,7,0,1,3,Qt::AlignCenter);

    ui.abas->clear();
    ui.abas->addTab(ui.aba_home, tr("Selecionar"));

    if(!isFullScreen() && !isMaximized())
        resize(560,300);

    setUpdatesEnabled(true);
}

void MainWindow::on_dial_valueChanged(int valor)
{
    QString qstr;
    qstr.setNum(valor);

    ui.campo_base->setText(qstr);
}

void MainWindow::on_slider_caracol_valueChanged(int valor)
{
    QString qstr;
    qstr.setNum(valor);

    ui.campo_caracol->setText(qstr);
}

void MainWindow::on_slider_ang_valueChanged(int valor)
{
    QString qstr;
    qstr.setNum(valor);

    ui.campo_ang->setText(qstr);
}

void MainWindow::on_slider_lin_valueChanged(int valor)
{
    QString qstr;
    qstr.setNum(valor);

    ui.campo_lin->setText(qstr);
}

void MainWindow::on_slider_dir_valueChanged(int valor)
{
    QString qstr;
    qstr.setNum(valor);

    ui.campo_dir->setModified(true);
    ui.campo_dir->setText(qstr);
}

void MainWindow::on_slider_esq_valueChanged(int valor)
{
    QString qstr;
    qstr.setNum(valor);

    ui.campo_esq->setModified(true);
    ui.campo_esq->setText(qstr);
}

void MainWindow::on_campo_max_dir_textChanged(QString qstr)
{
    valid_dir->setTop(qstr.toInt());
    valid_lin->setTop((qstr.toInt() + ui.slider_esq->maximum()) / 2);
    valid_lin->setBottom(-valid_lin->top());
    valid_ang->setTop(valid_lin->top() / LARGURA);
    valid_ang->setBottom(-valid_lin->top() / LARGURA);

    ui.slider_dir->setRange(0,qstr.toInt());
    ui.slider_lin->setRange(valid_lin->bottom(),valid_lin->top());
    ui.slider_ang->setRange(valid_ang->bottom(),valid_ang->top());
}

void MainWindow::on_campo_max_esq_textChanged(QString qstr)
{
    valid_esq->setTop(qstr.toInt());
    valid_lin->setTop((qstr.toInt() + ui.slider_dir->maximum()) / 2);
    valid_lin->setBottom(-valid_lin->top());
    valid_ang->setTop(valid_lin->top() / LARGURA);
    valid_ang->setBottom(-valid_lin->top() / LARGURA);

    ui.slider_esq->setRange(0,qstr.toInt());
    ui.slider_lin->setRange(valid_lin->bottom(),valid_lin->top());
    ui.slider_ang->setRange(valid_ang->bottom(),valid_ang->top());
}

void MainWindow::on_up_ang_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_ang->text().toFloat() + .1);

    ui.campo_ang->setText(qstr);
}

void MainWindow::on_up_lin_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_lin->text().toInt() + 1);

    ui.campo_lin->setText(qstr);
}

void MainWindow::on_up_dir_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_dir->text().toInt() + 1);

    if(qstr.toInt() <= ui.slider_dir->maximum())
        ui.campo_dir->setText(qstr);
}

void MainWindow::on_up_esq_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_esq->text().toInt() + 1);

    if(qstr.toInt() <= ui.slider_esq->maximum())
        ui.campo_esq->setText(qstr);
}

void MainWindow::on_up_base_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_base->text().toInt() + 1);

    ui.campo_base->setText(qstr);
}

void MainWindow::on_up_caracol_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_caracol->text().toInt() + 1);

    if(qstr.toInt() <= ui.slider_caracol->maximum())
        ui.campo_caracol->setText(qstr);
}

void MainWindow::on_down_ang_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_ang->text().toFloat() - .1);

    ui.campo_ang->setText(qstr);
}

void MainWindow::on_down_lin_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_lin->text().toInt() - 1);

    ui.campo_lin->setText(qstr);
}

void MainWindow::on_down_dir_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_dir->text().toInt() - 1);

    if(qstr.toInt() >= ui.slider_dir->minimum())
        ui.campo_dir->setText(qstr);
}

void MainWindow::on_down_esq_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_esq->text().toInt() - 1);

    if(qstr.toInt() >= ui.slider_esq->minimum())
        ui.campo_esq->setText(qstr);
}

void MainWindow::on_down_base_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_base->text().toInt() - 1);

    ui.campo_base->setText(qstr);
}

void MainWindow::on_down_caracol_pressed()
{
    QString qstr;
    qstr.setNum(ui.campo_caracol->text().toInt() - 1);

    if(qstr.toInt() >= ui.slider_caracol->minimum())
        ui.campo_caracol->setText(qstr);
}

void MainWindow::on_campo_esq_textChanged(QString qstr)
{       
    ROS_INFO("Mandou mudar ESQ para %.1f:",qstr.toFloat());

    if(!ui.campo_esq->isModified())
    {
        if(qstr.toFloat() < 0 && teleop.ang_esq)
        {
            ROS_INFO("--> ESQ: Clicando botao porque mandou negativo e tava pra frente;");

            on_bot_esq_clicked();
        }
        else if(qstr.toFloat() > 0 && !teleop.ang_esq)
        {
            ROS_INFO("--> ESQ: Clicando botao porque mandou positivo e tava pra tras;");
            on_bot_esq_clicked();
        }
    }

    if(qstr.toFloat() < 0)
    {
        ROS_INFO("--> ESQ: Tomando valor absoluto;");
        qstr.setNum(-qstr.toFloat());

        ui.campo_esq->blockSignals(true);
        ui.campo_esq->setText(qstr);
        ui.campo_esq->blockSignals(false);
    }

    ui.campo_esq->setModified(false);

    if(qstr.toFloat() > valid_esq->top())
    {
        ROS_INFO("--> ESQ: Ultrapassou o máximo. Constingindo...");

        qstr.setNum(valid_esq->top());
    }

    teleop.vel_esq = qstr.toFloat();
    teleop.comando_prop();

    ui.slider_esq->blockSignals(true);
    ui.slider_esq->setValue(teleop.vel_esq);
    ui.slider_esq->blockSignals(false);

    QString ang, lin;
    ang.setNum(teleop.vel_ang);
    lin.setNum(teleop.vel_lin);

    ROS_INFO("--> ESQ: atualizando ANG (%.3f) e LIN (%.1f)...",teleop.vel_ang,teleop.vel_lin);

    ui.campo_lin->setText(lin);
    ui.campo_ang->setText(ang);
}

void MainWindow::on_campo_dir_textChanged(QString qstr)
{
    ROS_INFO("Mandou mudar DIR para %.1f:",qstr.toFloat());

    if(!ui.campo_dir->isModified())
    {
        if(qstr.toFloat() < 0 && teleop.ang_dir)
        {
            ROS_INFO("--> DIR: Clicando botao porque mandou negativo e tava pra frente;");

            on_bot_dir_clicked();
        }
        else if(qstr.toFloat() > 0 && !teleop.ang_dir)
        {
            ROS_INFO("--> DIR: Clicando botao porque mandou positivo e tava pra tras;");
            on_bot_dir_clicked();
        }
    }

    if(qstr.toFloat() < 0)
    {
        ROS_INFO("--> DIR: Tomando valor absoluto;");

        qstr.setNum(-qstr.toFloat());

        ui.campo_dir->blockSignals(true);
        ui.campo_dir->setText(qstr);
        ui.campo_dir->blockSignals(false);
    }

    ui.campo_dir->setModified(false);

    if(qstr.toFloat() > valid_dir->top())
    {
        ROS_INFO("--> DIR: Ultrapassou o máximo. Constingindo...");
        qstr.setNum(valid_dir->top());
    }

    teleop.vel_dir = qstr.toFloat();
    teleop.comando_prop();

    ui.slider_dir->blockSignals(true);
    ui.slider_dir->setValue(teleop.vel_dir);
    ui.slider_dir->blockSignals(false);

    QString ang, lin;
    ang.setNum(teleop.vel_ang);
    lin.setNum(teleop.vel_lin);

    ROS_INFO("--> DIR: atualizando ANG (%.3f) e LIN (%.1f)...",teleop.vel_ang,teleop.vel_lin);

    ui.campo_ang->setText(ang);
    ui.campo_lin->setText(lin);
}

void MainWindow::on_campo_ang_textChanged(QString qstr)
{
    ROS_INFO("Mandou mudar ANG para %.3f:",qstr.toFloat());

    teleop.vel_ang = qstr.toFloat();
    teleop.atualiza_vel();

    ui.slider_ang->blockSignals(true);
    ui.slider_ang->setValue(teleop.vel_ang);
    ui.slider_ang->blockSignals(false);

    QString esq, dir;
    esq.setNum(teleop.vel_esq);
    dir.setNum(teleop.vel_dir);

    ROS_INFO("--> ANG: atualizando ESQ (%.1f) e DIR (%.1f)!",teleop.vel_esq,teleop.vel_dir);

    ui.campo_esq->setText(esq);
    ui.campo_dir->setText(dir);
}

void MainWindow::on_campo_lin_textChanged(QString qstr)
{
    ROS_INFO("Mandou mudar LIN para %.1f:",qstr.toFloat());

    teleop.vel_lin = qstr.toFloat();
    teleop.atualiza_vel();

    ui.slider_lin->blockSignals(true);
    ui.slider_lin->setValue(teleop.vel_lin);
    ui.slider_lin->blockSignals(false);

    QString esq, dir;
    esq.setNum(teleop.vel_esq);
    dir.setNum(teleop.vel_dir);

    ROS_INFO("--> LIN: atualizando ESQ (%.1f) e DIR (%.1f)!",teleop.vel_esq,teleop.vel_dir);

    ui.campo_esq->setText(esq);
    ui.campo_dir->setText(dir);
}

void MainWindow::on_campo_base_textChanged(QString qstr)
{
    if(qstr.toInt() < 0 || qstr.toInt() > 47)
    {
        qstr.setNum((qstr.toInt() + 48 ) % 48);
        std::cout << qstr.toStdString() << std::endl;
        ui.campo_base->setText(qstr);
        return;
    }

    teleop.base_pos = qstr.toInt();

    ui.dial->blockSignals(true);
    ui.dial->setValue(teleop.base_pos);
    ui.dial->blockSignals(false);

    teleop.comando_base();
}

void MainWindow::on_campo_caracol_textChanged(QString qstr)
{
    teleop.caracol_pos = qstr.toInt();

    ui.slider_caracol->blockSignals(true);
    ui.slider_caracol->setValue(teleop.caracol_pos);
    ui.slider_caracol->blockSignals(false);

    teleop.comando_caracol();
}

void MainWindow::on_vel_base_valueChanged(int valor)
{
    teleop.base_vel = valor;
    teleop.comando_base();
}

void MainWindow::on_vel_caracol_valueChanged(int valor)
{
    teleop.caracol_vel = valor;
    teleop.comando_caracol();
}

void MainWindow::base_dir()
{
    if(ui.opt_base_dir->isChecked())
        teleop.base_dir = 1;
    else if(ui.opt_base_0->isChecked())
        teleop.base_dir = 0;
    else if(ui.opt_base_esq->isChecked())
        teleop.base_dir = -1;

    teleop.comando_base();
}

void MainWindow::caracol_dir()
{
    if(ui.opt_caracol_dir->isChecked())
        teleop.caracol_dir = 1;
    else if(ui.opt_caracol_0->isChecked())
        teleop.caracol_dir = 0;
    else if(ui.opt_caracol_esq->isChecked())
        teleop.caracol_dir = -1;

    teleop.comando_caracol();
}

void MainWindow::on_bot_esq_clicked()
{    
    teleop.ang_esq = !teleop.ang_esq;

    ROS_INFO("----> Clicou botao ESQ! Agora esta para %s.",(teleop.ang_esq) ? "Frente" : "Tras");

    if(teleop.ang_esq)
        ui.bot_esq->setText(tr("Frente"));
    else
        ui.bot_esq->setText(tr("Trs").insert(2,225)); // á

    teleop.comando_prop();

    ROS_INFO("----> Botao ESQ atualizando ANG (%.3f) e LIN (%.1f).",teleop.vel_ang,teleop.vel_lin);

    QString ang, lin;
    ang.setNum(teleop.vel_ang);
    lin.setNum(teleop.vel_lin);

    ui.campo_lin->setText(lin);
    ui.campo_ang->setText(ang);
}

void MainWindow::on_bot_dir_clicked()
{
    teleop.ang_dir = !teleop.ang_dir;

    ROS_INFO("----> Clicou botao DIR! Agora esta para %s.",(teleop.ang_esq) ? "Frente" : "Tras");

    if(teleop.ang_dir)
        ui.bot_dir->setText(tr("Frente"));
    else
        ui.bot_dir->setText(tr("Trs").insert(2,225)); // á

    teleop.comando_prop();

    ROS_INFO("----> Botao DIR atualizando ANG (%.3f) e LIN (%.1f).",teleop.vel_ang,teleop.vel_lin);

    QString ang, lin;
    ang.setNum(teleop.vel_ang);
    lin.setNum(teleop.vel_lin);

    ui.campo_lin->setText(lin);
}

void MainWindow::on_bot_garra_clicked()
{
    teleop.comando_garra();

    if(teleop.estado_garra)
        ui.bot_garra->setText(tr("Fechar"));
    else
        ui.bot_garra->setText(tr("Abrir"));
}

void MainWindow::on_bot_panico_clicked()
{
    ui.campo_esq->setText(tr("0"));
    ui.campo_dir->setText(tr("0"));
    ui.campo_base->setText(tr("0"));
    ui.campo_caracol->setText(tr("0"));

    teleop.estado_garra = false;
    on_bot_garra_clicked();
}

void MainWindow::mostra_prop()
{
    QString qstr;

    qstr.setNum(monit.msg_prop.vel_esq.data);
    ui.painel_esq->setText(qstr);

    qstr.setNum(monit.msg_prop.vel_dir.data);
    ui.painel_dir->setText(qstr);

    qstr.setNum(monit.msg_prop.ang_esq.data);
    ui.painel_pos_esq->setText(qstr);

    qstr.setNum(monit.msg_prop.ang_dir.data);
    ui.painel_pos_dir->setText(qstr);

    qstr.setNum(monit.lin);
    ui.painel_lin->setText(qstr);

    qstr.setNum(monit.ang);
    ui.painel_ang->setText(qstr);

    ui.barra_esq->setValue(monit.msg_prop.vel_esq.data * 100/monit.prop_max);
    ui.barra_dir->setValue(monit.msg_prop.vel_dir.data * 100/monit.prop_max);

    ui.barra_lin->setValue(monit.lin * 100/monit.prop_max);
    ui.barra_ang->setValue(monit.ang * 100/monit.ang_max);

    pont_esq->angulo(-monit.msg_prop.ang_esq.data);
    pont_dir->angulo(monit.msg_prop.ang_dir.data);
}

void MainWindow::mostra_base()
{
    QString qstr;

    qstr.setNum(monit.msg_base.setpoint.data);
    ui.painel_base->setText(qstr);

    qstr.setNum(monit.msg_base.speed.data * monit.msg_base.dir.data);
    ui.painel_base_vel->setText(qstr);

    pont_base->angulo(monit.msg_base.setpoint.data * 180/48);
}

void MainWindow::mostra_caracol()
{
    QString qstr;

    qstr.setNum(monit.msg_caracol.setpoint.data);
    ui.painel_caracol->setText(qstr);

    qstr.setNum(monit.msg_caracol.speed.data * monit.msg_caracol.dir.data);
    ui.painel_caracol_vel->setText(qstr);

    ui.barra_caracol->setValue(monit.msg_caracol.setpoint.data * 100/256);
}

void MainWindow::mostra_garra()
{
    QPalette pal_cinza, pal_vermelha, pal_verde, pal_lilas;

    pal_cinza = pal_vermelha = pal_verde = pal_lilas = palette();

    pal_cinza.setColor(QPalette::WindowText, gray);
    pal_vermelha.setColor(QPalette::WindowText, red);
    pal_verde.setColor(QPalette::Base, QColor::fromRgb(170,255,127));
    pal_lilas.setColor(QPalette::Base, QColor::fromRgb(170,127,255));

    if(monit.msg_garra_in.data)
        ui.texto_dentro->setPalette(pal_vermelha);
    else
        ui.texto_dentro->setPalette(pal_cinza);

    if(monit.msg_garra_out.data)
        ui.texto_fora->setPalette(pal_vermelha);
    else
        ui.texto_fora->setPalette(pal_cinza);

    if(monit.msg_garra_open.data)
    {
        ui.texto_garra->setText("Aberta");
        ui.texto_garra->setPalette(pal_verde);
    }
    else
    {
        ui.texto_garra->setText("Fechada");
        ui.texto_garra->setPalette(pal_lilas);

    }
}

void MainWindow::mostra_imu()
{
    pont_imu->angulo(monit.msg_imu.data);
}

void MainWindow::mostra_ultrassom()
{
    QString qstr[4];

    for(int i = 0; i < 4; i++)
        if(monit.msg_ultrassom.data[i] != -1)
            qstr[i].setNum(monit.msg_ultrassom.data[i]);
        else
            qstr[i] = "...";

    ui.painel_ultr_frente->setText(qstr[0]);
    ui.painel_ultr_tras->setText(qstr[1]);
    ui.painel_ultr_esq->setText(qstr[2]);
    ui.painel_ultr_dir->setText(qstr[3]);
}

void MainWindow::mostra_bateria()
{
    QString qstr;

    qstr.setNum(monit.msg_bateria_v.data);
    ui.painel_tensao->setText(qstr);

    qstr.setNum(monit.msg_bateria_i.data);
    ui.painel_corrente->setText(qstr);
}

void MainWindow::mostra_camera()
{
    // Sei lá o que fazer aqui
}

Ponteiro::Ponteiro(QWidget *parent)
    : QWidget(parent),
      ang(0),
      cor(0, 127, 127, 191)
{}

void Ponteiro::paintEvent(QPaintEvent *)
{
    static const QPoint triang[3] = {
        QPoint(20, 70),
        QPoint(-20, 70),
        QPoint(0, -80)
    };

    int tam = qMin(width(), height());

    QPainter ptr(this);
    ptr.setRenderHint(QPainter::Antialiasing);
    ptr.translate(width() / 2, height() / 2);
    ptr.scale(tam/200., tam/200.);

    ptr.setPen(Qt::NoPen);
    ptr.setBrush(cor);

    ptr.save();
    ptr.rotate(ang);
    ptr.drawConvexPolygon(triang, 3);
    ptr.restore();

    ptr.setPen(cor);

    for (int i = 0; i < 8; ++i)
    {
        ptr.drawLine(88, 0, 96, 0);
        ptr.rotate(45.);
    }

    lower();
}

void Ponteiro::angulo(int novo_angulo)
{
    ang = novo_angulo;
    update();
}

void Ponteiro::colorir(QColor nova_cor)
{
    cor = nova_cor;
}

}  // namespace telemetria
