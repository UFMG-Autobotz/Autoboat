#include "arduino_class.hpp"

#ifndef BARCO_CLASS_H
#define BARCO_CLASS_H

#define PASSOS_MAX 48
#define MASTER_ID "MAST"
#define CHASSI_ID "BOAT"
#define CARACOL_ID "COL"

enum Stepper_state {direita = -1, desligado, esquerda};
enum MSG_state {M_A, M_AB, M_ABU, M_AC, M_ACU, M_AM, M_AMU, M_ARD, M_BC, M_BU, M_CC, M_G, M_GI, M_GO, M_Ibat, M_LA, M_LL, M_LV, M_P, M_SB, M_SC, M_U, M_Vbat};

typedef struct _Stepper {
	int setpoint, current;
	float speed;
	Stepper_state dir;
}Stepper;

class Barco_class {

private:
	static std::vector <float> ultrassom /*0-frente, 1-direita, 2-atras, 3-esquerda*/, mot_ang, mot_vel, /*0-esq, 1-dir*/ bloco, blocos_coord_oil, blocos_coord_port;
	static float v_bateria, i_tot, ang_init, ang_atual; //Vai depender do tipo de dado do bhryan
	static bool tem_bloco, garra_state, LED_l, LED_a, LED_v, but;
	static Stepper base, caracol;
	static std::vector<utils::Dict> msgs;
	static std::vector <Arduino_class> arduinos;

public:  
	Barco_class();
	static void init_barco();
	static void print_barco();

	static void add_arduino(std::string id, std::vector <std::string> smid, std::vector <std::string> rmid);
	static void send_arduinos_msgs();
	static void receive_arduinos_msgs();
	static bool connect_arduino(utils::Ports p, std::string id, double up_t);
	static bool disconnect_arduino(utils::Ports p);
	static double check_connection_state(std::string id);
	static void print_arduinos();

	static void set_msgs_list(std::vector<utils::Dict> m);
	static std::string get_msg_ID(std::string s);
	static bool check_port_state(utils::Ports p); // true == tem arduino conectado

	static void set_v_bat(float v);
	static float get_v_bat(){return v_bateria;}
	static void set_i_tot(float i);
	static float get_i_tot(){return i_tot;}

	static void set_LED_l(bool l);
	static bool get_LED_l() { return LED_l; }
	static void set_LED_a(bool l);
	static bool get_LED_a() { return LED_a; }
	static void set_LED_v(bool l);
	static bool get_LED_v() { return LED_v; }
	static void set_but(bool b);
	static bool get_but() { return but; }
	
	static int set_stepper(int stp, int cur);
	static int Rotate_stepper(int cur, int np, int dir);//vel em rpm = 1.25 / (s por passo) ou * (passos por s) -> .8 passos por s = 1 rpm -> (.8 = 360 graus / (60 s * 7.5 graus por passo)

	static void set_base(Stepper b);
	static void set_base_cmd(int sp, float sd, int d);
	static void set_base_current(int c);
	static Stepper get_base() { return base; }

	static void set_caracol(Stepper c);
	static void set_caracol_cmd(int sp, float sd, int d);
	static void set_caracol_current(int c);
	static Stepper get_caracol() { return caracol; }

	static void set_ultrassom(std::vector <float> dist);
	static std::vector <float> get_ultrassom() { return ultrassom; }

	static void set_mot_ang(std::vector <float> dist);
	static std::vector <float> get_mot_ang() { return mot_ang; }

	static void set_mot_vel(std::vector <float> dist);
	static std::vector <float> get_mot_vel() { return mot_vel; }

	static void set_blocos_coord_port(std::vector <float> bc);
	static std::vector <float> get_blocos_coord_port() { return blocos_coord_port; }

	static void set_blocos_coord_oil(std::vector <float> bc);
	static std::vector <float> get_blocos_coord_oil() { return blocos_coord_oil; }

	static void set_ang(float ang);
	static float get_ang() { return ang_atual; }

	static void set_ang_init(float ang);
	static float get_ang_init() { return ang_init; }

	static void set_block_state(bool bs);
	static bool get_block_state() { return tem_bloco; }

	static void set_garra_state(bool gs);
	static bool get_garra_state() { return garra_state; }
};

#endif