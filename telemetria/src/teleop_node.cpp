#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <telemetria/Stepper_msg.h>
#include <telemetria/Prop_msg.h>
#include <sstream>
#include "../include/telemetria/teleop_node.hpp"

namespace telemetria
{

TeleopNode::TeleopNode(int argc, char** argv) :
	init_argc(argc),
    init_argv(argv),
    estado_garra(true),
    vel_esq(0),
    ang_esq(true),
    vel_dir(0),
    ang_dir(true),
    vel_ang(0),
    vel_lin(0),
    base_pos(0),
    base_vel(60),
    base_dir(0),
    caracol_pos(0),
    caracol_vel(60),
    caracol_dir(0)
	{}

TeleopNode::~TeleopNode()
{
    if(ros::isStarted())
    {
      ros::shutdown();
      ros::waitForShutdown();
    }

    wait();
}

bool TeleopNode::init()
{
    ros::init(init_argc,init_argv,"teleoperacao");

    if (!ros::master::check())
		return false;

    ros::start();
    ros::NodeHandle n;

    base_pub = n.advertise<telemetria::Stepper_msg>("autoboat/caracol/base_stepper_cmd",200);
    caracol_pub = n.advertise<telemetria::Stepper_msg>("autoboat/caracol/caracol_stepper_cmd",200);
    prop_pub = n.advertise<telemetria::Prop_msg>("autoboat/prop",350);
    garra_pub = n.advertise<std_msgs::Bool>("autoboat/garra/open",80);

    start();

    return true;
}


void TeleopNode::run()
{
    while (ros::ok());

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown();
}


void TeleopNode::comando_garra()
{
    estado_garra = !estado_garra;

    std_msgs::Bool msg;
    msg.data = estado_garra;

    garra_pub.publish(msg);
}

#define sg_vel(d) ((ang_##d) ? vel_##d : -vel_##d)

void TeleopNode::comando_prop()
{
    telemetria::Prop_msg msg;

    msg.vel_esq.data = fabs(vel_esq);
    msg.vel_dir.data = fabs(vel_dir);
    msg.ang_esq.data = vel_esq >= 0;
    msg.ang_dir.data = vel_dir >= 0;

    prop_pub.publish(msg);

    vel_ang = (sg_vel(dir) - sg_vel(esq)) / LARGURA;
    vel_lin = (sg_vel(dir) + sg_vel(esq)) / 2;
}

void TeleopNode::atualiza_vel()
{
    float dif;
    dif = vel_ang * LARGURA;

    vel_esq = vel_lin - dif/2;
    vel_dir = vel_lin + dif/2;

    comando_prop();
}

void TeleopNode::comando_base()
{
    telemetria::Stepper_msg msg;

    msg.setpoint.data = base_pos;
    msg.speed.data = base_vel;
    msg.dir.data = base_dir;

    base_pub.publish(msg);
}

void TeleopNode::comando_caracol()
{
    telemetria::Stepper_msg msg;

    msg.setpoint.data = caracol_pos;
    msg.speed.data = caracol_vel;
    msg.dir.data = caracol_dir;

    caracol_pub.publish(msg);
}

}  // namespace telemetria
