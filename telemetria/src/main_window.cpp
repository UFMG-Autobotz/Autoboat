#include <QtGui>
#include <QMessageBox>
#include "../include/telemetria/main_window.hpp"

namespace telemetria
{

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , teleop(argc,argv)
    , monit(argc,argv)
{
    ui.setupUi(this); // Conectar os sinais dos objetos da UI aos callbacks da forma on_...();

    setWindowIcon(QIcon(":/images/logo.png"));  // Define a logo da Autobotz como ícone da janela

    while(!monit.init())    // Espera o nó ser inicializado
    {
        QMessageBox msgBox;
        msgBox.setText("Nao foi possível iniciar o no de telemetria.\nVerifique se o rosmaster foi inicializado.");
        msgBox.exec();
    }

    // Manter apenas a aba 0 (boas-vindas)
    ui.abas->removeTab(2);
    ui.abas->removeTab(1);

/*  // Tentativa falha de inserir uma imagem

    QPixmap img_auto(":/images/auto.png");
    img_auto = img_auto.scaled(QSize(100,100));

    QLabel* label_auto = new QLabel(ui.aba_home);
    label_auto->setPixmap(img_auto);
    ui.layout_home->addWidget(label_auto,6,0,Qt::AlignCenter);
*/
    // Conectar sinais aos slots
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

    // Constrói os quatro ponteiros giratórios da GUI
    pont_esq = new Ponteiro(ui.frame_prop);
    pont_dir = new Ponteiro(ui.frame_prop);
    pont_imu = new Ponteiro(ui.frame_imu);
    pont_base = new Ponteiro(ui.frame_base);

    // Adiciona os ponteiros construídos à GUI
    ui.layout_base->addWidget(pont_base,1,0,1,2);
    ui.layout_imu->addWidget(pont_imu,2,1);
    ui.layout_prop->addWidget(pont_esq,4,1);
    ui.layout_prop->addWidget(pont_dir,4,2);

    // Traz os painéis (números indicadores) para frente dos ponteiros na GUI
    ui.painel_base->raise();
    ui.painel_imu->raise();
}

MainWindow::~MainWindow() {}

void MainWindow::sair()
{
    close();    // Fecha a janela
}

void MainWindow::iniciar_autonav()
{
//    ROS_INFO("Iniciando no de navegação autonoma...");

//    int i = system("rosrun turtlesim turtlesim_node");

//    ROS_INFO("%i",i);

    setUpdatesEnabled(false);   // Não atualiza a aba enquanto ela está sendo modificada

    ui.abas->clear();   // Fecha todas as abas
    ui.abas->addTab(ui.aba_telemetr, tr("Monitoramento"));  // Adiciona a aba de monitoramento

    if(!isFullScreen() && !isMaximized())   // Redimensiona se não estiver maximizado ou tela cheia
        resize(840,500);

    setUpdatesEnabled(true);    // Atualiza as modificações feitas na janela
}

void MainWindow::iniciar_joystick()
{
//    ROS_INFO("Iniciando no de controle via joystick");
//    int i = system("roslaunch autoboat_joystick autoboat_joystick.launch");
//    ROS_INFO("%i",i);

    setUpdatesEnabled(false);   // Não atualiza a aba enquanto ela está sendo modificada

    ui.abas->clear();   // Fecha todas as abas
    ui.abas->addTab(ui.aba_telemetr, tr("Monitoramento"));  // Adiciona a aba de monitoramento

    if(!isFullScreen() && !isMaximized())   // Redimensiona se não estiver maximizado ou tela cheia
        resize(840,500);

    setUpdatesEnabled(true);    // Atualiza as modificações feitas na janela
}

void MainWindow::iniciar_teleop()   // Inicializa a o nó e a aba de teleoperação
{
    if(!teleop.init())  // Inicializa o nó
    {
        QMessageBox msgBox;
        msgBox.setText("Nao foi possível iniciar o no de teleop.\nVerifique se o rosmaster foi inicializado.");
        msgBox.exec();

        return;
    }

    ROS_INFO("Iniciando no de teleop");

    setUpdatesEnabled(false);   // Não atualiza a aba enquanto ela está sendo modificada

    ui.abas->clear();   // Fecha todas as abas
    ui.abas->addTab(ui.aba_teleop, tr("Controle")); // Adiciona a aba de teleoperação
    ui.abas->addTab(ui.aba_telemetr, tr("Monitoramento"));  // Adiciona a aba de monituramento
    ui.abas->setCurrentIndex(0);    // Muda o foco para a primeira aba (teleoperação)

    // Cria objetos para restringir os valores digitados pelo usuário nos paineis de entrada
    valid_base = new QIntValidator(0,48,ui.campo_base);
    valid_caracol = new QIntValidator(0,100,ui.campo_caracol);
    valid_dir = new QDoubleValidator(0,50,1,ui.campo_dir);
    valid_esq = new QDoubleValidator(0,50,1,ui.campo_esq);
    valid_ang = new QDoubleValidator(-.625,.625,3,ui.campo_ang);
    valid_lin = new QDoubleValidator(-50,50,1,ui.campo_lin);

    // Associa os objetos aos respectivos painéis de entrada
    ui.campo_base->setValidator(valid_base);
    ui.campo_caracol->setValidator(valid_caracol);
    ui.campo_ang->setValidator(valid_ang);
    ui.campo_lin->setValidator(valid_lin);
    ui.campo_dir->setValidator(valid_dir);
    ui.campo_esq->setValidator(valid_esq);
    ui.campo_max_dir->setValidator(new QIntValidator(0,1e6,ui.campo_max_dir));
    ui.campo_max_esq->setValidator(new QIntValidator(0,1e6,ui.campo_max_esq));

    // Conectar sinais aos slots
    QObject::connect(ui.bot_sair, SIGNAL(clicked()), this, SLOT(sair()));
    QObject::connect(ui.opt_base_esq, SIGNAL(toggled(bool)), this, SLOT(base_dir()));
    QObject::connect(ui.opt_base_dir, SIGNAL(toggled(bool)), this, SLOT(base_dir()));
    QObject::connect(ui.opt_caracol_esq, SIGNAL(toggled(bool)), this, SLOT(caracol_dir()));
    QObject::connect(ui.opt_caracol_dir, SIGNAL(toggled(bool)), this, SLOT(caracol_dir()));
    
    if(!isFullScreen() && !isMaximized())   // Redimensiona se não estiver maximizado ou tela cheia
        resize(840,500);

    setUpdatesEnabled(true);    // Atualiza as modificações feitas na janela
}

void MainWindow::trocar_operacao()  // Chamada quando clica o botão "Trocar modo de operação"
{
    setUpdatesEnabled(false);   // Não atualiza a aba enquanto ela está sendo modificada

    // Muda o texto principal da aba de boas vindas
    ui.texto_boasvindas->setText(tr("Vejo que mudou de ideia, Autoboticista.\nDeseja trocar o modo de operação ou sair?"));

    // Muda o a fonte e o texto do botão de sair e o reposiciona
    ui.bot_nevermind->setFont(ui.bot_teleop->font());
    ui.bot_nevermind->setText(tr("Quero ir embora. Obrigado."));
    ui.layout_home->addWidget(ui.bot_nevermind,7,0,1,3,Qt::AlignCenter);

    ui.abas->clear();   // Fecha todas as abas
    ui.abas->addTab(ui.aba_home, tr("Selecionar")); // Volta para a aba de boas-vindas com novo nome

    if(!isFullScreen() && !isMaximized())   // Redimensiona se não estiver maximizado ou tela cheia
        resize(560,300);

    setUpdatesEnabled(true);    // Atualiza as modificações feitas na janela
}

// Funções que atualizam os campos de texto quando os sliders (ou o dial) são arrastados:

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
    muda_lin_ang(vel_lin,valor,true);
}

void MainWindow::on_slider_lin_valueChanged(int valor)
{
    muda_lin_ang(valor,vel_ang,true);
}

void MainWindow::on_slider_dir_valueChanged(int valor)
{
    muda_esq_dir(teleop.vel_esq,valor,true);
}

void MainWindow::on_slider_esq_valueChanged(int valor)
{
    muda_esq_dir(valor,teleop.vel_dir,true);
}

// Funções que atualizam os limites dos sliders:

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

// Funções que lidam com os botões de incrementar e decrementar:

void MainWindow::on_up_ang_pressed()
{
    if(vel_ang < ui.slider_ang->maximum())
        muda_lin_ang(vel_lin,vel_ang+.1,true);
}

void MainWindow::on_up_lin_pressed()
{
    if(vel_lin < ui.slider_lin->maximum())
        muda_lin_ang(vel_lin+1,vel_ang,true);
}

void MainWindow::on_up_dir_pressed()
{
    if(teleop.vel_dir < ui.slider_dir->maximum())
        muda_esq_dir(teleop.vel_esq,teleop.vel_dir+1,true);
}

void MainWindow::on_up_esq_pressed()
{
    if(teleop.vel_esq < ui.slider_esq->maximum())
        muda_esq_dir(teleop.vel_esq+1,teleop.vel_dir,true);
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
    if(vel_ang > ui.slider_ang->minimum())
        muda_lin_ang(vel_lin,vel_ang-.1,true);
}

void MainWindow::on_down_lin_pressed()
{
    if(vel_lin > ui.slider_lin->minimum())
        muda_lin_ang(vel_lin-1,vel_ang,true);
}

void MainWindow::on_down_dir_pressed()
{
    if(teleop.vel_dir > ui.slider_dir->minimum())
        muda_esq_dir(teleop.vel_esq,teleop.vel_dir-1,true);
}

void MainWindow::on_down_esq_pressed()
{
    if(teleop.vel_esq > ui.slider_esq->minimum())
        muda_esq_dir(teleop.vel_esq-1,teleop.vel_dir,true);
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

// Campos numéricos das velocidades dos propulsores:

void MainWindow::on_campo_esq_textChanged(QString qstr)
{       
    muda_esq_dir(qstr.toFloat(),teleop.vel_dir,true);
}

void MainWindow::on_campo_dir_textChanged(QString qstr)
{
    muda_esq_dir(teleop.vel_esq,qstr.toFloat(),true);
}

// Campos numéricos das velocidades linear e angular:

void MainWindow::on_campo_lin_textChanged(QString qstr)
{
    muda_lin_ang(qstr.toFloat(),vel_ang,true);
}

void MainWindow::on_campo_ang_textChanged(QString qstr)
{
    muda_lin_ang(vel_lin,qstr.toFloat(),true);
}

// Campos numéricos da posição dos steppers:

void MainWindow::on_campo_base_textChanged(QString qstr)
{
    if(qstr.toInt() < 0 || qstr.toInt() > 47)
    {
        qstr.setNum((qstr.toInt() + 48 ) % 48); // Garante que o valor é positivo e está entre 0 e 47
        ui.campo_base->setText(qstr);
        return;
    }

    teleop.base_pos = qstr.toInt(); // Informa ao nó o novo valor da posição da base

    // Atualiza o dial (sem que ele mande sinal para atualizar novamente o campo numérico)
    ui.dial->blockSignals(true);
    ui.dial->setValue(teleop.base_pos);
    ui.dial->blockSignals(false);

    teleop.comando_base();  // Manda o nó publicar no tópico da base
}

void MainWindow::on_campo_caracol_textChanged(QString qstr)
{
    teleop.caracol_pos = qstr.toInt();  // Informa ao nó o novo valor da posição do caracol

    // Atualiza o slider (sem que ele mande sinal para atualizar novamente o campo numérico)
    ui.slider_caracol->blockSignals(true);
    ui.slider_caracol->setValue(teleop.caracol_pos);
    ui.slider_caracol->blockSignals(false);

    teleop.comando_caracol();   // Manda o nó publicar no tópico do caracol
}

// Campos numéricos da velocidade dos steppers:

void MainWindow::on_vel_base_valueChanged(int valor)
{
    teleop.base_vel = valor;    // Informa ao nó o novo valor da velocidade da base
    teleop.comando_base();  // Manda o nó publicar no tópico da base
}

void MainWindow::on_vel_caracol_valueChanged(int valor)
{
    teleop.caracol_vel = valor; // Informa ao nó o novo valor da velocidade do caracol
    teleop.comando_caracol();   // Manda o nó publicar no tópico do caracol
}

// Caixas de seleção da direção (anti-horário, livre, horário)

void MainWindow::base_dir()
{
    // Verifica qual opção está marcada e informa ao nó
    if(ui.opt_base_dir->isChecked())
        teleop.base_dir = 1;
    else if(ui.opt_base_0->isChecked())
        teleop.base_dir = 0;
    else if(ui.opt_base_esq->isChecked())
        teleop.base_dir = -1;

    teleop.comando_base();  // Manda o nó publicar no tópico da base
}

void MainWindow::caracol_dir()
{
    // Verifica qual opção está marcada e informa ao nó
    if(ui.opt_caracol_dir->isChecked())
        teleop.caracol_dir = 1;
    else if(ui.opt_caracol_0->isChecked())
        teleop.caracol_dir = 0;
    else if(ui.opt_caracol_esq->isChecked())
        teleop.caracol_dir = -1;

    teleop.comando_caracol();   // Manda o nó publicar no tópico do caracol
}

// LEDs

void MainWindow::on_led_azul_clicked(bool estado)
{
    teleop.estado_led_azul = estado;
    teleop.comando_leds();

    QPalette pal = palette();

    if(estado)
        pal.setColor(QPalette::WindowText, QColor::fromHsv(240,255,220)); // Azul claro
    else
        pal.setColor(QPalette::WindowText, QColor::fromHsv(240,255,50)); // Azul escuro

    ui.led_azul->setPalette(pal);
}

void MainWindow::on_led_laranja_clicked(bool estado)
{
    teleop.estado_led_laranja = estado;
    teleop.comando_leds();

    QPalette pal = palette();

    if(estado)
        pal.setColor(QPalette::WindowText, QColor::fromHsv(30,255,220)); // Laranja claro
    else
        pal.setColor(QPalette::WindowText, QColor::fromHsv(30,255,50)); // Laranja escuro

    ui.led_laranja->setPalette(pal);
}

void MainWindow::on_led_verde_clicked(bool estado)
{
    teleop.estado_led_verde = estado;
    teleop.comando_leds();

    QPalette pal = palette();

    if(estado)
        pal.setColor(QPalette::WindowText, QColor::fromHsv(30,255,220)); // Verde claro
    else
        pal.setColor(QPalette::WindowText, QColor::fromHsv(30,255,50)); // Verde escuro

    ui.led_verde->setPalette(pal);
}

// Botões da direção dos propulsores:

void MainWindow::on_bot_esq_clicked()
{    
    if(teleop.ang_esq == 0)
        teleop.ang_esq = 180;
    else
        teleop.ang_esq = 0;

    muda_esq_dir(teleop.vel_esq,teleop.vel_dir,true);
}

void MainWindow::on_bot_dir_clicked()
{
    if(teleop.ang_dir == 0)
        teleop.ang_dir = 180;
    else
        teleop.ang_dir = 0;

    muda_esq_dir(teleop.vel_esq,teleop.vel_dir,true);
}

// Botão da garra
void MainWindow::on_bot_garra_clicked()
{
    teleop.estado_garra = !teleop.estado_garra; // Alterna entre aberta e fechada

    if(teleop.estado_garra)
        ui.bot_garra->setText(tr("Fechar"));
    else
        ui.bot_garra->setText(tr("Abrir"));

    teleop.comando_garra(); // Manda o nó publicar no tópico da garra
}

// Botão de pânico
void MainWindow::on_bot_panico_clicked()
{
    // Zera tudo
    muda_lin_ang(0,0,true);
    ui.campo_base->setText(tr("0"));
    ui.campo_caracol->setText(tr("0"));

    // Garante que a garra esteja aberta
    teleop.estado_garra = false;
    ui.bot_garra->click();
}

// Funções que atualizam os valores na aba de monitoramento:

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

    ui.barra_lin->setValue(50 + monit.lin * 50/monit.prop_max);
    ui.barra_ang->setValue(50 + monit.ang * 50/monit.ang_max);

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
        if(monit.msg_ultrassom.data[i] > 0)     // Confere se não está fora de alcance
            qstr[i].setNum(monit.msg_ultrassom.data[i]);
        else
            qstr[i] = "...";    // Imprime reticências caso esteja fora de alcance

    ui.painel_ultr_frente->setText(qstr[0]);
    ui.painel_ultr_dir->setText(qstr[1]);
    ui.painel_ultr_tras->setText(qstr[2]);
    ui.painel_ultr_esq->setText(qstr[3]);
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

// Específico para a propulsão:

#define sgn(d) ((teleop.ang_##d == 0) ? d : -d) // Sinal positivo ou negativo dependendo da posição do propulsor

void MainWindow::muda_esq_dir(float esq, float dir, bool continua)
{
    teleop.vel_esq = esq;
    teleop.vel_dir = dir;

    // Bloqueia sinais para evitar loop infinito
    ui.slider_esq->blockSignals(true);
    ui.slider_dir->blockSignals(true);
    ui.campo_esq->blockSignals(true);
    ui.campo_dir->blockSignals(true);

    // Atualiza sliders
    ui.slider_esq->setValue(esq);
    ui.slider_dir->setValue(dir);

    QString qstr;

    // Atualiza campos numéricos
    qstr.setNum(esq);
    ui.campo_esq->setText(qstr);
    qstr.setNum(dir);
    ui.campo_dir->setText(qstr);

    atualiza_posicao();

    // Volta sinais
    ui.slider_esq->blockSignals(false);
    ui.slider_dir->blockSignals(false);
    ui.campo_esq->blockSignals(false);
    ui.campo_dir->blockSignals(false);

    teleop.comando_prop();  // Manda o nó publicar no tópico da propulsão

    if(continua)    // Não executa quando chamado via muda_lin_ang (para evitar loop)
    {
        float lin = (sgn(dir) + sgn(esq)) / 2;
        float ang = (sgn(dir) - sgn(esq)) / LARGURA;

        muda_lin_ang(lin,ang,false);
    }
}

void MainWindow::muda_lin_ang(float lin, float ang, bool continua)
{
    vel_lin = lin;
    vel_ang = ang;

    // Bloqueia sinais para evitar loop infinito
    ui.slider_lin->blockSignals(true);
    ui.slider_ang->blockSignals(true);
    ui.campo_lin->blockSignals(true);
    ui.campo_ang->blockSignals(true);

    // Atualiza sliders
    ui.slider_lin->setValue(lin);
    ui.slider_ang->setValue(ang);

    QString qstr;

    // Atualiza campos numéricos
    qstr.setNum(lin);
    ui.campo_lin->setText(qstr);
    qstr.setNum(ang);
    ui.campo_ang->setText(qstr);

    // Volta sinais
    ui.slider_lin->blockSignals(false);
    ui.slider_ang->blockSignals(false);
    ui.campo_lin->blockSignals(false);
    ui.campo_ang->blockSignals(false);

    if(continua)    // Não executa quando chamado via muda_esq_dir (para evitar loop)
    {
        float esq = lin - ang * LARGURA / 2;
        float dir = lin + ang * LARGURA / 2;

        teleop.ang_esq = (esq >= 0) ? 0 : 180;
        teleop.ang_dir = (dir >= 0) ? 0 : 180;

        muda_esq_dir(fabs(esq), fabs(dir), false);
    }
}

void MainWindow::atualiza_posicao()
{
    if(teleop.ang_esq == 0)
        ui.bot_esq->setText(tr("Frente"));
    else
        ui.bot_esq->setText(tr("Trs").insert(2,0xE1)); // Gambiarra temporária para exibir o caractere 'á' (U+00E1)

    if(teleop.ang_dir == 0)
        ui.bot_dir->setText(tr("Frente"));
    else
        ui.bot_dir->setText(tr("Trs").insert(2,0xE1));
}

}  // namespace telemetria

