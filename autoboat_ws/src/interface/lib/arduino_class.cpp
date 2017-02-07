#include "arduino_class.hpp"

Arduino_class::Arduino_class(std::string id, std::vector <std::string> smid, std::vector <std::string> rmid){
	ID = id;
	send_msgs_ID = smid;
	receive_msgs_ID = rmid;
	conectado = false;
	p.nome = "empty";
	p.n = -1;
	up_since = -1;
}

void Arduino_class::set_port(utils::Ports _p){
	p = _p;
	// p.serial = new Rs232_class(p.n);
	conectado = true;
}

void Arduino_class::set_up_time(double t){
	up_since = t;
}

void Arduino_class::disconect(){
	conectado = false;
	p.nome = "empty";
	p.n = -1;
	// delete p.serial;
}

std::string Arduino_class::receive_msgs(){
	return p.serial->receive();
}

void Arduino_class::send_msgs(std::string s){
	p.serial->send(s);
}