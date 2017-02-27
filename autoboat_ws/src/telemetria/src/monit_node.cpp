#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <autoboat_msgs/Stepper_msg.h>
#include <autoboat_msgs/Prop_msg.h>
#include "../include/telemetria/monit_node.hpp"

namespace telemetria
{

MonitNode::MonitNode(int argc, char** argv) :   // Inicializa variáveis
    init_argc(argc),
    init_argv(argv)
{
    prop_max = 50;
    ang_max = 2 * prop_max / LARGURA;
}

MonitNode::~MonitNode()
{
    if(ros::isStarted())    // Verifica se ros::start() foi chamada
    {
      ros::shutdown();  // Pede o ROS para desligar
      ros::waitForShutdown();   // Espera o desligamento
    }

    wait(); // Espera run() retornar
}

bool MonitNode::init()
{
    ros::init(init_argc,init_argv,"monitoramento");  // Inicializa ROS

    if (!ros::master::check())  // Confere se o roscore foi iniciado
        return false;

    ros::start();       // Inicializa nó
    ros::NodeHandle n;

    // Subscribers
    base_sub       = n.subscribe("autoboat/caracol/base_stepper_current",1,&MonitNode::base_cb,this);
    caracol_sub    = n.subscribe("autoboat/caracol/caracol_stepper_current",1,&MonitNode::caracol_cb,this);
    prop_sub       = n.subscribe("autoboat/prop",1,&MonitNode::prop_cb,this);
    garra_open_sub = n.subscribe("autoboat/garra/open",1,&MonitNode::garra_open_cb,this);
    garra_in_sub   = n.subscribe("autoboat/garra/IR_in",1,&MonitNode::garra_in_cb,this);
    garra_out_sub  = n.subscribe("autoboat/garra/IR_out",1,&MonitNode::garra_out_cb,this);
    imu_sub        = n.subscribe("autoboat/angulo",1,&MonitNode::imu_cb,this);
    bateria_i_sub  = n.subscribe("autoboat/power/v_bat",1,&MonitNode::bateria_i_cb,this);
    bateria_v_sub  = n.subscribe("autoboat/power/i_bat",1,&MonitNode::bateria_v_cb,this);
    ultrassom_sub  = n.subscribe("autoboat/sensors/ultrassons",1,&MonitNode::ultrassom_cb,this);
//  camera_sub     = n.subscribe("autoboat/camera/rgb/image_color",1,&MonitNode::camera_cb,this);

    start();    // Inicializa thread e chama run()

    return true;
}

void MonitNode::run()   // Comportamento da thread
{
    while (ros::ok())
        ros::spin();    // Roda os callbaks

    std::cout << "Fim da operação de telemetria." << std::endl;

    Q_EMIT rosShutdown(); // Sinaliza que o nó foi desativado
}

// Callbacks:

void MonitNode::prop_cb(const autoboat_msgs::Prop_msg &msg)
{
    msg_prop = msg;

    // Atualiza valores de velocidade linear e angular baseados na posição e potência dos propulsores.
    // Esses valores não são publicados, servem apenas para visualização na aba de monitoramento.

    lin = (msg.vel_dir.data * cos(msg.ang_dir.data*M_PI/180) + msg.vel_esq.data * cos(msg.ang_esq.data*M_PI/180)) / 2;
    ang = (msg.vel_dir.data * cos(msg.ang_dir.data*M_PI/180) - msg.vel_esq.data * cos(msg.ang_esq.data*M_PI/180)) / LARGURA;

    Q_EMIT atual_prop();
}

void MonitNode::base_cb(const autoboat_msgs::Stepper_msg& msg)
{
  msg_base = msg;
  Q_EMIT atual_base();
}

void MonitNode::caracol_cb(const autoboat_msgs::Stepper_msg& msg)
{
  msg_caracol = msg;
  Q_EMIT atual_caracol();
}

void MonitNode::garra_open_cb(const std_msgs::Bool& msg)
{
  msg_garra_open = msg;
  Q_EMIT atual_garra();
}

void MonitNode::garra_in_cb(const std_msgs::Bool& msg)
{
  msg_garra_in = msg;
  Q_EMIT atual_garra();
}

void MonitNode::garra_out_cb(const std_msgs::Bool& msg)
{
  msg_garra_out = msg;
  Q_EMIT atual_garra();
}

void MonitNode::imu_cb(const std_msgs::Float32& msg)
{
  msg_imu = msg;
  Q_EMIT atual_imu();
}

void MonitNode::ultrassom_cb(const std_msgs::Float32MultiArray& msg)
{
  msg_ultrassom = msg;
  Q_EMIT atual_ultrassom();
}

void MonitNode::bateria_i_cb(const std_msgs::Float32& msg)
{
  msg_bateria_i = msg;
  Q_EMIT atual_bateria();
}

void MonitNode::bateria_v_cb(const std_msgs::Float32& msg)
{
  msg_bateria_v = msg;
  Q_EMIT atual_bateria();
}

}
