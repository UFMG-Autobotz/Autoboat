#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <autoboat_msgs/Stepper_msg.h>
#include <autoboat_msgs/Prop_msg.h>
#include "../include/telemetria/teleop_node.hpp"

namespace telemetria
{

TeleopNode::TeleopNode(int argc, char** argv) : // Inicializa variáveis
	init_argc(argc),
    init_argv(argv),
    estado_garra(true),
    vel_esq(0),
    ang_esq(0),
    vel_dir(0),
    ang_dir(0),
    base_pos(0),
    base_vel(60),
    base_dir(0),
    caracol_pos(0),
    caracol_vel(60),
    caracol_dir(0)
	{}

TeleopNode::~TeleopNode()
{
    if(ros::isStarted())    // Verifica se ros::start() foi chamada
    {
      ros::shutdown();  // Pede o ROS para desligar
      ros::waitForShutdown();   // Espera o desligamento
    }

    wait(); // Espera run() retornar
}

bool TeleopNode::init()
{
    ros::init(init_argc,init_argv,"teleoperacao");  // Inicializa ROS

    if (!ros::master::check())  // Confere se o roscore foi iniciado
		return false;

    ros::start();       // Inicializa nó
    ros::NodeHandle n;

    // Publishers
    base_pub    = n.advertise <autoboat_msgs::Stepper_msg>("autoboat/caracol/base_stepper_cmd",20);
    caracol_pub = n.advertise <autoboat_msgs::Stepper_msg>("autoboat/caracol/caracol_stepper_cmd",20);
    prop_pub    = n.advertise <autoboat_msgs::Prop_msg>   ("autoboat/prop",35);
    garra_pub   = n.advertise <std_msgs::Bool>            ("autoboat/garra/open",8);

    start();    // Inicializa thread e chama run()

    return true;
}

void TeleopNode::run()      // Comportamento da thread
{
    while (ros::ok());      // Espera desativação do nó

    Q_EMIT rosShutdown();   // Sinaliza que o nó foi desativado
}

// Para publicar nos tópicos, a aba de teleoperação chama as funções a seguir.

void TeleopNode::comando_garra()
{
    std_msgs::Bool msg;

    msg.data = estado_garra;

    garra_pub.publish(msg);
}

void TeleopNode::comando_prop()
{
    autoboat_msgs::Prop_msg msg;

    msg.vel_esq.data = vel_esq;
    msg.vel_dir.data = vel_dir;
    msg.ang_esq.data = ang_esq;
    msg.ang_dir.data = ang_dir;

    prop_pub.publish(msg);
}

void TeleopNode::comando_base()
{
    autoboat_msgs::Stepper_msg msg;

    msg.setpoint.data = base_pos;
    msg.speed.data = base_vel;
    msg.dir.data = base_dir;

    base_pub.publish(msg);
}

void TeleopNode::comando_caracol()
{
    autoboat_msgs::Stepper_msg msg;

    msg.setpoint.data = caracol_pos;
    msg.speed.data = caracol_vel;
    msg.dir.data = caracol_dir;

    caracol_pub.publish(msg);
}

}  // namespace telemetria
