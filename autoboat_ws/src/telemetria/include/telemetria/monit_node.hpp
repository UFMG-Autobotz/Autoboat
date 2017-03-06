#ifndef telemetria_MONITNODE_HPP_
#define telemetria_MONITNODE_HPP_

#include <QThread>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <autoboat_msgs/Stepper_msg.h>
#include <autoboat_msgs/Prop_msg.h>

#define LARGURA 80  // Largura do barco

namespace telemetria
{

class MonitNode : public QThread
{
    Q_OBJECT

public:

    MonitNode(int argc, char** argv);
    virtual ~MonitNode();

    // Funções da thread
    bool init();
    void run();

    // Armazenam os valores atuais de cada informação do barco
    autoboat_msgs::Prop_msg msg_prop;
    std_msgs::Int32 msg_base, msg_caracol;
    std_msgs::Bool msg_garra_open, msg_garra_in, msg_garra_out;
    std_msgs::Float32 msg_imu, msg_bateria_i, msg_bateria_v;
    std_msgs::Float32MultiArray msg_ultrassom;

    // Valores para exibição na GUI
    int prop_max, ang_max;
    float ang, lin;

Q_SIGNALS:

    // Sinais do Qt enviados para a GUI poder atualizar os valores
    void atual_prop();
    void atual_base();
    void atual_caracol();
    void atual_garra();
    void atual_imu();
    void atual_ultrassom();
    void atual_bateria();
    void atual_camera();
    void rosShutdown();

private:

    int init_argc;
    char** init_argv;

    // Subscribers
    ros::Subscriber prop_sub, base_sub, caracol_sub, garra_open_sub,
                    garra_in_sub, garra_out_sub, imu_sub, ultrassom_sub,
                    bateria_i_sub, bateria_v_sub, camera_sub;

    // Callbacks
    void prop_cb      (const autoboat_msgs::Prop_msg&);
    void base_cb      (const std_msgs::Int32 &);
    void caracol_cb   (const std_msgs::Int32 &);
    void garra_open_cb(const std_msgs::Bool&);
    void garra_in_cb  (const std_msgs::Bool&);
    void garra_out_cb (const std_msgs::Bool&);
    void imu_cb       (const std_msgs::Float32&);
    void ultrassom_cb (const std_msgs::Float32MultiArray&);
    void bateria_i_cb (const std_msgs::Float32&);
    void bateria_v_cb (const std_msgs::Float32&);
//  void camera_cb    (const sensor_msgs::Image&);
};

}  // namespace telemetria

#endif /* telemetria_MONITNODE_HPP_ */
