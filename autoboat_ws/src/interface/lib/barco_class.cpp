#include "barco_class.hpp"

std::vector <float> Barco_class::ultrassom, Barco_class::mot_ang, Barco_class::mot_vel, Barco_class::blocos_coord_port, Barco_class::blocos_coord_oil;
float Barco_class::v_bateria, Barco_class::i_tot, Barco_class::ang_init, Barco_class::ang_atual; //Vai depender do tipo de dado do bhryan
bool Barco_class::tem_bloco, Barco_class::garra_state, Barco_class::LED_l, Barco_class::LED_a, Barco_class::LED_v, Barco_class::but;
Stepper Barco_class::base, Barco_class::caracol;
std::vector<utils::Dict> Barco_class::msgs;
std::vector <Arduino_class> Barco_class::arduinos;

Barco_class::Barco_class(){
}
void Barco_class::init_barco(){
	ultrassom.clear();
	mot_ang.clear();
	mot_vel.clear();
	ultrassom.push_back(0); ultrassom.push_back(1); ultrassom.push_back(2); ultrassom.push_back(3);
    mot_ang.push_back(0); mot_ang.push_back(0);
    mot_vel.push_back(30); mot_vel.push_back(30);
	v_bateria = 0;
	i_tot = 1;
	ang_atual = 2;
	tem_bloco = garra_state = LED_l = false;
	LED_a = LED_v = but =true;
	base.setpoint = 1;
	base.speed = 0;
	base.dir = direita;
	base.current = 2;
	caracol.setpoint = 3;
	caracol.speed = 4;
	caracol.dir = direita;
	caracol.current = 5;
}

void Barco_class::print_barco(){
	std::vector <float > ult = Barco_class::get_ultrassom();
	std::cout << "LED_laranja:\t\t\t" << Barco_class::get_LED_l() << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "LED_azul:\t\t\t" << Barco_class::get_LED_a() << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "LED_verde:\t\t\t" << Barco_class::get_LED_v() << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "button:\t\t\t\t" << Barco_class::get_but() << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "v_bat:\t\t\t\t" << Barco_class::get_v_bat() << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "i_total:\t\t\t" << Barco_class::get_i_tot() << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "ardumaster_up:\t\t\t" << (Barco_class::check_connection_state(MASTER_ID) != -1) << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "ardumaster_up_since:\t\t" << Barco_class::check_connection_state(MASTER_ID) << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "arduboat_up:\t\t\t" << (Barco_class::check_connection_state(CHASSI_ID) != -1) << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "arduboat_up_since:\t\t" << Barco_class::check_connection_state(CHASSI_ID) << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "arducol_up:\t\t\t" << (Barco_class::check_connection_state(CARACOL_ID) != -1) << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "arducol_up_since:\t\t" << Barco_class::check_connection_state(CARACOL_ID) << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "ultrassons:\t\t\t" << ult[0] << " " << ult[1] << " " << ult[2] << " " << ult[3] << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "angulo:\t\t\t\t" << Barco_class::get_ang() << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "IR_in:\t\t\t\t" << Barco_class::get_block_state() << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "base_stepper_current:\t\t" << Barco_class::get_base().current << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;
	std::cout << "caracol_stepper_current:\t" << Barco_class::get_caracol().current << std::endl;
	std::cout << "------------------------------------------------"<< std::endl;

}

void Barco_class::add_arduino(std::string id, std::vector <std::string> smid, std::vector <std::string> rmid){
	arduinos.push_back(Arduino_class(id, smid, rmid));
}

void Barco_class::send_arduinos_msgs(){
	for (int i = 0; i < arduinos.size(); ++i){
		if (arduinos[i].get_connection_state()){
			std::vector <std::string> s_m_ID = arduinos[i].get_send_ID();
			std::string s = Data_msg;
			s+= DIV_MSG + arduinos[i].get_ID();
			for (int j = 0; j < s_m_ID.size(); ++j){
				s += DIV_MSG + s_m_ID[j] + INF_MSG;
				switch(atoi(get_msg_ID(s_m_ID[j]).c_str() ) ){
					case M_A:
						break;
					case M_AB:
						s += utils::numTostr((int) (check_connection_state(CHASSI_ID) != -1) );
						break;
					case M_ABU:
						s += utils::numTostr(check_connection_state(CHASSI_ID));
						break;
					case M_AC:
						s += utils::numTostr((int) (check_connection_state(CARACOL_ID) != -1) );
						break;
					case M_ACU:
						s += utils::numTostr(check_connection_state(CARACOL_ID));
						break;
					case M_AM:
						s += utils::numTostr((int) (check_connection_state(MASTER_ID) != -1) );
						break;
					case M_AMU:
						s += utils::numTostr(check_connection_state(MASTER_ID));
						break;
					case M_ARD:
						s += arduinos[i].get_ID();
						break;
					case M_BC:
						break;
					case M_BU:
						break;
					case M_CC:
						break;
					case M_G:
						s += utils::numTostr((int) Barco_class::get_garra_state());
						break;
					case M_GI:
						break;
					case M_GO:
						break;
					case M_Ibat:
						break;
					case M_LA:
                        s += utils::numTostr((int) Barco_class::get_LED_a());
						break;
					case M_LL:
                        s += utils::numTostr((int) Barco_class::get_LED_l());
						break;
					case M_LV:
                        s += utils::numTostr((int) Barco_class::get_LED_v());
						break;
					case M_P:
						s += utils::numTostr(Barco_class::get_mot_vel()[0]) + ":";
						s += utils::numTostr(Barco_class::get_mot_vel()[1]) + ":";
						s += utils::numTostr(Barco_class::get_mot_ang()[0]) + ":";
						s += utils::numTostr(Barco_class::get_mot_ang()[1]);
						break;
					case M_SB:
						s += utils::numTostr(Barco_class::get_base().setpoint) + ":";
						s += utils::numTostr(Barco_class::get_base().speed) + ":";
						s += utils::numTostr((int) Barco_class::get_base().dir);
						break;
					case M_SC:
						s += utils::numTostr(Barco_class::get_caracol().setpoint) + ":";
						s += utils::numTostr(Barco_class::get_caracol().speed) + ":";
						s += utils::numTostr((int) Barco_class::get_caracol().dir);
						break;
					case M_U:
						break;
					case M_Vbat:
						break;
				}
			}
			s += END_MSG;
			arduinos[i].send_msgs(s);
			// arduinos[i].send_msgs("super batatas com frango");
		}
	}
}

void Barco_class::receive_arduinos_msgs(){
	static std::vector<float> ult;
	static std::vector<std::string> s_vec;
	for (int i = 0; i < arduinos.size(); ++i){
		if (arduinos[i].get_connection_state()){
			std::vector <std::string> r_m_ID = arduinos[i].get_receive_ID();

			std::string msg_received = arduinos[i].receive_msgs();
			std::cout << "Got message from " << arduinos[i].get_ID() << ":" << msg_received << "that's it"<< std::endl << std::endl;
			std::vector<utils::Dict> parsed_msg = utils::parse_ard_msg(msg_received);
			for (int j = 0; j < parsed_msg.size(); ++j){
				parsed_msg[j].value.erase(std::remove(parsed_msg[j].value.begin(), parsed_msg[j].value.end(), REPLACE_CHAR), parsed_msg[j].value.end());
				switch(atoi(parsed_msg[j].key.c_str())){
					case M_A:
						Barco_class::set_ang(atof(parsed_msg[j].value.c_str()));
						break;
					case M_AB:
						break;
					case M_ABU:
						break;
					case M_AC:
						break;
					case M_ACU:
						break;
					case M_AM:
						break;
					case M_AMU:
						break;
					case M_ARD:
						if (parsed_msg[j].value != arduinos[i].get_ID())
							arduinos[i].disconect();
						break;
					case M_BC:
						Barco_class::set_base_current(atoi(parsed_msg[j].value.c_str()));
						break;
					case M_BU:
						Barco_class::set_but(parsed_msg[j].value != "0");
						break;
					case M_CC:
						Barco_class::set_caracol_current(atoi(parsed_msg[j].value.c_str()));
						break;
					case M_G:
						break;
					case M_GI:
						Barco_class::set_block_state(parsed_msg[j].value != "0");
						break;
					case M_GO:
						break;
					case M_Ibat:
						Barco_class::set_i_tot(atof(parsed_msg[j].value.c_str()));
						break;
/*
                    case M_LA:
                        Barco_class::set_LED_a(parsed_msg[j].value != "0");
						break;
					case M_LL:
                        Barco_class::set_LED_l(parsed_msg[j].value != "0");
						break;
					case M_LV:
                        Barco_class::set_LED_v(parsed_msg[j].value != "0");
                        break;
*/
					case M_P:
						break;
					case M_SB:
						break;
					case M_SC:
						break;
					case M_U:
						ult.clear();
						s_vec.clear();
						s_vec = utils::parse_vector_msg(parsed_msg[j].value);
						for (int i = 0; i < s_vec.size(); ++i)
							ult.push_back(atof(s_vec[i].c_str()));
						Barco_class::set_ultrassom(ult);
						break;
					case M_Vbat:
						Barco_class::set_v_bat(atof(parsed_msg[j].value.c_str()));
						break;
				}
			}
		}
	}
}

bool Barco_class::connect_arduino(utils::Ports p, std::string id, double up_t){
	for (int i = 0; i < arduinos.size(); ++i)
		if (id == arduinos[i].get_ID()){
			arduinos[i].set_port(p);
			arduinos[i].set_up_time(up_t);
			return true;
		}
	return false;
}

bool Barco_class::disconnect_arduino(utils::Ports p){
	for (int i = 0; i < arduinos.size(); ++i)
		if (p.nome == arduinos[i].get_port_info().nome){
			arduinos[i].disconect();
			return true;
		}
	return false;
}

double Barco_class::check_connection_state(std::string id){
	for (int i = 0; i < arduinos.size(); ++i)
		if (id == arduinos[i].get_ID())
			if (arduinos[i].get_connection_state())
				return arduinos[i].get_up_time();
			else
				return -1;
}

void Barco_class::print_arduinos(){
	std::vector <std::string> s_vec;
	for (int i = 0; i < arduinos.size(); ++i){
		std::cout << "Arduino " << arduinos[i].get_ID() << ":" << std::endl;
		std::cout << "\tPort: " << arduinos[i].get_port_info().nome << ", number " << arduinos[i].get_port_info().n << std::endl;
		std::cout << "\tConnection state: " << arduinos[i].get_connection_state() << std::endl;
		std::cout << "\tLast up time: " << arduinos[i].get_up_time() << std::endl;
		std::cout << "\tReceive:" << std::endl;
		s_vec.clear();
		s_vec = arduinos[i].get_receive_ID();
		for (int j = 0; j < s_vec.size(); ++j)
			std::cout << "\t\t" << s_vec[j] << std::endl;
		std::cout << "\tSend" << std::endl;
		s_vec.clear();
		s_vec = arduinos[i].get_send_ID();
		for (int j = 0; j < s_vec.size(); ++j)
			std::cout << "\t\t" << s_vec[j] << std::endl;
		std::cout << std::endl << std::endl;
	}
}

void Barco_class::set_msgs_list(std::vector<utils::Dict> m){
	msgs = m;
}

std::string Barco_class::get_msg_ID(std::string s){
	for (int i = 0; i < msgs.size(); ++i)
		if (msgs[i].key == s)
			return msgs[i].value;
	std::cout << "msg not found" << std::endl;
	exit(1);
}

bool Barco_class::check_port_state(utils::Ports p){
	for (int i = 0; i < arduinos.size(); ++i)
		if (p.nome == arduinos[i].get_port_info().nome)
			return true;
	return false;
}

void Barco_class::set_v_bat(float v){
	v_bateria = v;
}
void Barco_class::set_i_tot(float i){
	i_tot = i;
}

void Barco_class::set_LED_l(bool l){
	LED_l = l;
}

void Barco_class::set_LED_a(bool l){
	LED_a = l;
}

void Barco_class::set_LED_v(bool l){
	LED_v = l;
}

void Barco_class::set_but(bool b){
	but = b;
}

int Barco_class::set_stepper(int stp, int cur){
	int dir = (int)(abs(stp - cur) < PASSOS_MAX - abs(stp - cur) ? (cur - stp)/abs(stp - cur) : (stp - cur)/abs(stp - cur)); //1-horário
	return dir;
}

int Barco_class::Rotate_stepper(int cur, int np, int dir){ //vel em rpm = 1.25 / (s por passo) ou * (passos por s) -> .8 passos por s = 1 rpm -> (.8 = 360 graus / (60 s * 7.5 graus por passo)
	int stp = ( cur + dir*np );
	stp = stp < PASSOS_MAX && stp > 0 ? stp : (stp < PASSOS_MAX ? 0 : PASSOS_MAX); 
	return stp;
}

void Barco_class::set_caracol_cmd(int sp, float sd, int d){
	caracol.setpoint = sp;
	caracol.speed = sd;
	caracol.dir = static_cast<Stepper_state>(d);
}

void Barco_class::set_caracol_current(int c){
	caracol.current = c;
}

void Barco_class::set_caracol(Stepper c){
	caracol = c;
}

void Barco_class::set_base_cmd(int sp, float sd, int d){
	base.setpoint = sp;
	base.speed = sd;
	base.dir = static_cast<Stepper_state>(d);
}

void Barco_class::set_base_current(int c){
	base.current = c;
}

void Barco_class::set_base(Stepper b){
	base = b;
}

void Barco_class::set_ultrassom(std::vector <float> dist){
	ultrassom = dist;
}

void Barco_class::set_blocos_coord_port(std::vector <float> bc){
	blocos_coord_port = bc;
}

void Barco_class::set_blocos_coord_oil(std::vector <float> bc){
	blocos_coord_oil = bc;
}

void Barco_class::set_mot_ang(std::vector <float> ang){
	mot_ang = ang;
}

void Barco_class::set_mot_vel(std::vector <float> vel){
	mot_vel = vel;
}

void Barco_class::set_ang(float ang){
	ang_atual = ang;
}

void Barco_class::set_ang_init(float ang){
	ang_init = ang;
}

void Barco_class::set_block_state(bool bs){
	tem_bloco = bs;
}

void Barco_class::set_garra_state(bool gs){
	garra_state = gs;
}
