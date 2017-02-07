#ifndef telemetria_TELEOPNODE_HPP_
#define telemetria_TELEOPNODE_HPP_

#include <ros/ros.h>
#include <QThread>

#define LARGURA 80  // Largura do barco

namespace telemetria
{

class TeleopNode : public QThread
{
    Q_OBJECT

public:

    TeleopNode(int argc, char** argv);
    virtual ~TeleopNode();

    // Variáveis alteradas pela aba de teleoperação
    bool estado_garra;
    int base_vel, base_pos, base_dir, caracol_vel, caracol_pos, caracol_dir;
    float vel_dir, vel_esq, ang_dir, ang_esq;

    // Funções da thread
    bool init();
	void run();
    void tchau();

    // Comunicação entre a aba de teleoperação e os publishers
    void comando_garra();
    void comando_prop();
    void comando_base();
    void comando_caracol();

Q_SIGNALS:

    void rosShutdown();

private:

	int init_argc;
	char** init_argv;

    ros::Publisher base_pub, caracol_pub, garra_pub, prop_pub;
};

}  // namespace telemetria

#endif /* telemetria_TELEOPNODE_HPP_ */
