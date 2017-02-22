#include "utils.hpp"

#ifndef ARDUINO_CLASS_H
#define ARDUINO_CLASS_H

//#define WAIT_TIME_us 70000

#define Data_msg "Dt_msg"
#define Id_msg "Id_msg"
// #define Id_msg "60 char messagesssssssssssssss60 char messagesssssssssssssss"//"Id_msg"

class Arduino_class {

private:
	std::string ID;
	std::vector <std::string> send_msgs_ID;
	std::vector <std::string> receive_msgs_ID;
	utils::Ports p;
	bool conectado;
	double up_since;

public:  
	Arduino_class(std::string id, std::vector <std::string> smid, std::vector <std::string> rmid);

	std::string get_ID() { return ID; }

	void set_port(utils::Ports _p);
	utils::Ports get_port_info() { return p; }

	void disconect();
	bool get_connection_state() { return conectado; }

	void set_up_time(double t);
	double get_up_time() { return up_since; }

	std::vector <std::string> get_send_ID() { return send_msgs_ID; }
	std::vector <std::string> get_receive_ID() { return receive_msgs_ID; }

	std::string receive_msgs();
	void send_msgs(std::string s);
};

#endif
