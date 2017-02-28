#include "../lib/ros_functions.hpp"

#define PRINT_INTERVAL 10

int main(int argc, char **argv){
	ros::init(argc, argv, "autoboat_hal");
	ros::NodeHandle nh;

	ros::Subscriber subG = nh.subscribe("/autoboat/garra/open", 1, ros_func::sub::open);
	ros::Subscriber subP = nh.subscribe("/autoboat/prop", 1, ros_func::sub::prop);
	ros::Subscriber subSB = nh.subscribe("/autoboat/caracol/base_stepper_cmd", 1, ros_func::sub::base_stepper_cmd);
	ros::Subscriber subSC = nh.subscribe("/autoboat/caracol/caracol_stepper_cmd", 1, ros_func::sub::caracol_stepper_cmd);
	ros::Subscriber subLA = nh.subscribe("/autoboat/interface/LED_azul", 1, ros_func::sub::LED_azul);
	ros::Subscriber subLL = nh.subscribe("/autoboat/interface/LED_laranja", 1, ros_func::sub::LED_laranja);
	ros::Subscriber subLV = nh.subscribe("/autoboat/interface/LED_verde", 1, ros_func::sub::LED_verde);

	ros::Publisher pubA = nh.advertise<std_msgs::Float32>("/autoboat/angulo", 1);
	ros::Publisher pubAB = nh.advertise<std_msgs::Bool>("/autoboat/diagnostics/arduboat_up", 1);
	ros::Publisher pubABU = nh.advertise<std_msgs::Time>("/autoboat/diagnostics/arduboat_up_since", 1);
	ros::Publisher pubAC = nh.advertise<std_msgs::Bool>("/autoboat/diagnostics/arducol_up", 1);
	ros::Publisher pubACU = nh.advertise<std_msgs::Time>("/autoboat/diagnostics/arducol_up_since", 1);
	ros::Publisher pubAM = nh.advertise<std_msgs::Bool>("/autoboat/diagnostics/ardumaster_up", 1);
	ros::Publisher pubAMU = nh.advertise<std_msgs::Time>("/autoboat/diagnostics/ardumaster_up_since", 1);
	ros::Publisher pubBC = nh.advertise<std_msgs::Int32>("/autoboat/caracol/base_stepper_current", 1);
	ros::Publisher pubBU = nh.advertise<std_msgs::Bool>("/autoboat/interface/button", 1);
	ros::Publisher pubCC = nh.advertise<std_msgs::Int32>("/autoboat/caracol/caracol_stepper_current", 1);
	ros::Publisher pubGI = nh.advertise<std_msgs::Bool>("/autoboat/garra/IR_in", 1);
	ros::Publisher pubGO = nh.advertise<std_msgs::Bool>("/autoboat/garra/IR_out", 1);
	ros::Publisher pubIbat = nh.advertise<std_msgs::Float32>("/autoboat/power/i_total", 1);
/*
	ros::Publisher pubLA = nh.advertise<std_msgs::Bool>("/autoboat/interface/LED_azul", 1);
	ros::Publisher pubLL = nh.advertise<std_msgs::Bool>("/autoboat/interface/LED_laranja", 1);
	ros::Publisher pubLV = nh.advertise<std_msgs::Bool>("/autoboat/interface/LED_verde", 1);
*/
	ros::Publisher pubU = nh.advertise<std_msgs::Float32MultiArray>("/autoboat/ultrassons", 1);
	ros::Publisher pubVbat = nh.advertise<std_msgs::Float32>("/autoboat/power/v_bat", 1);

	//ros::Rate loop_rate(2);
	Barco_class::init_barco();

	std::string caminho (argv[0]);
	caminho.replace(caminho.rfind("devel"), caminho.npos, "src/interface/configs/");

	std::string serial_filename = caminho + "Serial_ports.txt";
	std::string msgs_filename = caminho + "ID_MSGS.txt";
	std::string arduinos_filename = caminho + "Arduinos.txt";

	std::vector<utils::Ports> p_vec = utils::get_port_list(serial_filename);
	std::vector<utils::Dict> m_vec = utils::get_msg_list(msgs_filename), a_vec = utils::get_arduino_list(arduinos_filename);

	std::string id;
	std::vector <std::string> smid, rmid;

	for (int i = 0; i < p_vec.size(); ++i){
		std::cout << p_vec[i].n << " - " <<p_vec[i].nome << std::endl;
	}

	for (int i = 0; i < m_vec.size(); ++i){
		std::cout << m_vec[i].key << " - " << m_vec[i].value << std::endl;
	}
	Barco_class::set_msgs_list(m_vec);

	for (int i = 0; i <= a_vec.size(); ++i){

		if(a_vec[i].key == "Arduino_ID" || i == a_vec.size()){
			if(!id.empty())
				Barco_class::add_arduino(id, smid, rmid);
			if(i < a_vec.size())
				id = a_vec[i].value;
			smid.clear();
			rmid.clear();
		}
		else if(a_vec[i].key == "add_send_message")
			smid.push_back(a_vec[i].value);
		else if(a_vec[i].key == "add_receive_message")
			rmid.push_back(a_vec[i].value);

		// if(i < a_vec.size())
			// std::cout << a_vec[i].key << " - " << a_vec[i].value << std::endl;
	}

	// std::vector<Rs232_class*> Rs232_vector;
	// for (int i = 0; i < p_vec.size(); ++i)
	// 	Rs232_vector.push_back(new Rs232_class);

	for (int i = 0; i < p_vec.size(); ++i)
		p_vec[i].serial = new Rs232_class;


	std::vector<utils::Dict> connection_log[p_vec.size()];
	utils::Dict d_con;
	double con_t;

	double start_time = ros::Time::now().toSec();
	std::cout << "Run Time: " << ros::Time::now().toSec() - start_time << std::endl;
	std::cout << "Ports:" << std::endl;
	for (int i = 0; i < p_vec.size(); ++i)
		std::cout << "\t" << p_vec[i].nome << ": " << (utils::path_exist(p_vec[i].nome) ? "Existe" : "Nao existe" ) << std::endl;
	std::cout << std::endl;
	Barco_class::print_arduinos();
	Barco_class::print_barco();
	
	time_t start, end;
	time(&start);

	int delay_envio, delay_recebimento;

	while(ros::ok()){

		nh.param("autoboat/HAL/delay_envio", delay_envio, 100000);
		nh.param("autoboat/HAL/delay_recebimento", delay_recebimento, 120000);

		for (int i = 0; i < p_vec.size(); ++i){
			if(utils::path_exist(p_vec[i].nome)){
				// if(!Rs232_vector[i]->get_comport_state())
				// 	Rs232_vector[i]->open_comport(p_vec[i].n);
				if(!p_vec[i].serial->get_comport_state())
					p_vec[i].serial->open_comport(p_vec[i].n);

				if(Barco_class::check_port_state(p_vec[i])){
				}
				else{
					////// Rs232_class* serial = new Rs232_class();
					// Rs232_vector[i]->send(Id_msg);
					p_vec[i].serial->send(Id_msg);
					usleep(delay_recebimento);
					// std::vector<utils::Dict> parsed_msg = utils::parse_ard_msg(Rs232_vector[i]->receive());
					std::vector<utils::Dict> parsed_msg = utils::parse_ard_msg(p_vec[i].serial->receive());
					std::string ard_ID = parsed_msg.size() > 0 ? parsed_msg[0].value : "_";//receive response
					if(ard_ID == "_")
						std::cout << "No response on port " << p_vec[i].nome << std::endl;
					else
						std::cout << "Got response on port " << p_vec[i].nome << ", ID = " << ard_ID << std::endl;
					con_t = ros::Time::now().toSec() - start_time;
					// usleep(WAIT_TIME_us);
					if (Barco_class::connect_arduino(p_vec[i], ard_ID, con_t)){
						d_con.key = ard_ID;
						d_con.value = con_t;
						connection_log[i].push_back(d_con);
					}
				}
			}
			else{
				// Rs232_vector[i]->close_comport();
				p_vec[i].serial->close_comport();
				if(Barco_class::check_port_state(p_vec[i])){
					if (Barco_class::disconnect_arduino(p_vec[i])){
						con_t = ros::Time::now().toSec() - start_time;
						d_con.key = "disconnected";
						d_con.value = utils::numTostr(con_t);
						connection_log[i].push_back(d_con);
						//Barco_class::init_barco();
					}
				}
			}
		}

		Barco_class::send_arduinos_msgs();
		usleep(delay_envio);
		Barco_class::receive_arduinos_msgs();
		usleep(delay_recebimento);

		time(&end);
		if( end - start > PRINT_INTERVAL){
			std::cout << "Run Time: " << ros::Time::now().toSec() - start_time << std::endl;
			std::cout << "Ports:" << std::endl;
			for (int i = 0; i < p_vec.size(); ++i)
				std::cout << "\t" << p_vec[i].nome << ": " << (utils::path_exist(p_vec[i].nome) ? "Existe" : "Nao existe" ) << std::endl;
			std::cout << std::endl;
			Barco_class::print_arduinos();
			Barco_class::print_barco();
			time(&start);
		}
/*
		ros_func::pub::LED_laranja(pubLL);
		ros_func::pub::LED_azul(pubLA);
		ros_func::pub::LED_verde(pubLV);
*/
		ros_func::pub::button(pubBU);
		ros_func::pub::v_bat(pubVbat);
		ros_func::pub::i_total(pubIbat);
		ros_func::pub::IR_out(pubGO);
		ros_func::pub::ardumaster_up(pubAM);
		ros_func::pub::ardumaster_up_since(pubAMU);
		ros_func::pub::arduboat_up(pubAB);
		ros_func::pub::arduboat_up_since(pubABU);
		ros_func::pub::arducol_up(pubAC);
		ros_func::pub::arducol_up_since(pubACU);
		ros_func::pub::ultrassons(pubU);
		ros_func::pub::angulo(pubA);
		ros_func::pub::IR_in(pubGI);
		ros_func::pub::base_stepper_current(pubBC);
		ros_func::pub::caracol_stepper_current(pubCC);

		//loop_rate.sleep();
		ros::spinOnce();
	}
	for (int i = 0; i < p_vec.size(); ++i)
		delete p_vec[i].serial;
		// delete Rs232_vector[i];
	// for (int i = 0; i < p_vec.size(); ++i){
	// 	utils::vector_to_file("connection_log_" +  utils::numTostr(i) + ".txt", connection_log[i]);
	// }
	return 0;
}
