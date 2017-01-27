#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <telemetria/Stepper_msg.h>
#include <telemetria/Prop_msg.h>
#include "../include/telemetria/monit_node.hpp"

namespace telemetria
{

MonitNode::MonitNode(int argc, char** argv) :
    init_argc(argc),
    init_argv(argv)
{
    prop_max = 50;
    ang_max = 2 * prop_max / LARGURA;
}

MonitNode::~MonitNode()
{
    if(ros::isStarted())
    {
      ros::shutdown();
      ros::waitForShutdown();
    }

    wait();
}

bool MonitNode::init()
{
    ros::init(init_argc,init_argv,"monitoramento");

    if (!ros::master::check())
        return false;

    ros::start();
    ros::NodeHandle n;

    base_sub = n.subscribe("autoboat/caracol/base_stepper_current",1,&MonitNode::base_cb,this);
    caracol_sub = n.subscribe("autoboat/caracol/caracol_stepper_current",1,&MonitNode::caracol_cb,this);
    prop_sub = n.subscribe("autoboat/prop",1,&MonitNode::prop_cb,this);
    garra_open_sub = n.subscribe("autoboat/garra/open",1,&MonitNode::garra_open_cb,this);
    garra_in_sub = n.subscribe("autoboat/garra/IR_in",1,&MonitNode::garra_in_cb,this);
    garra_out_sub = n.subscribe("autoboat/garra/IR_out",1,&MonitNode::garra_out_cb,this);
    imu_sub = n.subscribe("autoboat/angulo",1,&MonitNode::imu_cb,this);
    //camera_sub = n.subscribe("autoboat/camera/rgb/image_color",1,&MonitNode::camera_cb,this);
    bateria_i_sub = n.subscribe("autoboat/power/v_bat",1,&MonitNode::bateria_i_cb,this);
    bateria_v_sub = n.subscribe("autoboat/power/i_bat",1,&MonitNode::bateria_v_cb,this);
    ultrassom_sub = n.subscribe("autoboat/sensors/ultrassons",1,&MonitNode::ultrassom_cb,this);

    start();

    return true;
}

void MonitNode::run()
{
    while (ros::ok())
        ros::spin();

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown();
}

void MonitNode::prop_cb(const telemetria::Prop_msg &msg)
{
    msg_prop = msg;

    lin = msg.vel_dir.data * cos(msg.ang_dir.data*M_PI/180) + msg.vel_esq.data * cos(msg.ang_esq.data*M_PI/180);
    ang = (msg.vel_dir.data * cos(msg.ang_dir.data*M_PI/180) - msg.vel_esq.data * cos(msg.ang_esq.data*M_PI/180)) / LARGURA;

    Q_EMIT atual_prop();
}

void MonitNode::base_cb(const telemetria::Stepper_msg& msg)
{
  msg_base = msg;
  Q_EMIT atual_base();
}

void MonitNode::caracol_cb(const telemetria::Stepper_msg& msg)
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
