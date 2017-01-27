#ifndef telemetria_TELEOPNODE_HPP_
#define telemetria_TELEOPNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>

#define LARGURA 80

namespace telemetria
{

class TeleopNode : public QThread
{
    Q_OBJECT

public:

    TeleopNode(int argc, char** argv );
    virtual ~TeleopNode();

    bool init();
    void tchau();
	void run();

    void comando_garra();
    void comando_prop();
    void atualiza_vel();
    void comando_base();
    void comando_caracol();

    bool estado_garra, ang_dir, ang_esq;
    int base_vel, base_pos, base_dir, caracol_vel, caracol_pos, caracol_dir;
    float vel_dir, vel_esq, vel_ang, vel_lin;

Q_SIGNALS:

    void rosShutdown();

private:

	int init_argc;
	char** init_argv;

    ros::Publisher base_pub, caracol_pub, garra_pub, prop_pub;
};

}  // namespace telemetria

#endif /* telemetria_TELEOPNODE_HPP_ */
