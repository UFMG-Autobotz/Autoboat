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
    bool estado_garra, estado_led_laranja, estado_led_azul, estado_led_verde;
    int base_pos, base_dir, caracol_pos, caracol_dir;
    float vel_dir, vel_esq, ang_dir, ang_esq, base_vel, caracol_vel;

    // Funções da thread
    bool init();
	void run();
    void tchau();

    // Comunicação entre a aba de teleoperação e os publishers
    void comando_garra();
    void comando_prop();
    void comando_base();
    void comando_caracol();
    void comando_leds();
    void autoriza_autonav(bool);
    void autoriza_joystick(bool);

Q_SIGNALS:

    void rosShutdown();

private:

	int init_argc;
	char** init_argv;

    ros::Publisher base_pub, caracol_pub, garra_pub, prop_pub, led_A_pub, led_L_pub, led_V_pub;
};

}  // namespace telemetria

#endif /* telemetria_TELEOPNODE_HPP_ */
