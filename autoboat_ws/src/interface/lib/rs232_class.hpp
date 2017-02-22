#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include <unistd.h>
#include "rs232lib.h"

#define REPLACE_CHAR '#'
#define FINAL_CHAR '!'
// #define SPLIT_CHAR '?'
// #define MAX_LENGTH 16
#define send_wait 100

#ifndef RS232_CLASS_H
#define RS232_CLASS_H

class Rs232_class {

private:
	int cport_nr,	//serial == 24 ou 25 (ACM0 ou ACM1) "/dev/ttyACM0"
	bdrate;		/* 9600 baud */
	char mode[4];
	unsigned char buf[4096];
	bool port_opened;

public:  
	Rs232_class();
	Rs232_class(int p, int r = 9600);
	~Rs232_class();
	void open_comport(int p);
	void close_comport();
	void set_baud_rate(int r);
	bool get_comport_state() { return port_opened; }
	template <typename V>
	void send(V var);
	std::string receive();
	unsigned char* get_buf() { return buf; }
};

#endif
