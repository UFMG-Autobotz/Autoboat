#ifndef telemetria_MONITNODE_HPP_
#define telemetria_MONITNODE_HPP_

#include <ros/ros.h>
#include <QThread>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <telemetria/Stepper_msg.h>
#include <telemetria/Prop_msg.h>

#define LARGURA 80

namespace telemetria
{

class MonitNode : public QThread
{
    Q_OBJECT

public:

    MonitNode(int argc, char** argv);
    virtual ~MonitNode();

    bool init();
    void run();

    telemetria::Prop_msg msg_prop;
    telemetria::Stepper_msg msg_base, msg_caracol;
    std_msgs::Bool msg_garra_open, msg_garra_in, msg_garra_out;
    std_msgs::Float32 msg_imu, msg_bateria_i, msg_bateria_v;
    std_msgs::Float32MultiArray msg_ultrassom;

    int prop_max, ang_max;
    float ang, lin;

Q_SIGNALS:

    void rosShutdown();
    void atual_prop();
    void atual_base();
    void atual_caracol();
    void atual_garra();
    void atual_imu();
    void atual_ultrassom();
    void atual_bateria();
    void atual_camera();

private:

    int init_argc;
    char** init_argv;

    ros::Subscriber prop_sub, base_sub, caracol_sub, garra_open_sub,
                    garra_in_sub, garra_out_sub, imu_sub, ultrassom_sub,
                    bateria_i_sub, bateria_v_sub, camera_sub;

    void prop_cb(const telemetria::Prop_msg& msg);
    void base_cb(const telemetria::Stepper_msg& msg);
    void caracol_cb(const telemetria::Stepper_msg& msg);
    void garra_open_cb(const std_msgs::Bool& msg);
    void garra_in_cb(const std_msgs::Bool& msg);
    void garra_out_cb(const std_msgs::Bool& msg);
    void imu_cb(const std_msgs::Float32& msg);
    void ultrassom_cb(const std_msgs::Float32MultiArray& msg);
    void bateria_i_cb(const std_msgs::Float32& msg);
    void bateria_v_cb(const std_msgs::Float32& msg);
    //void camera_cb();
};

}  // namespace telemetria

#endif /* telemetria_MONITNODE_HPP_ */
