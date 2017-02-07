#include "rs232_class.hpp"
#include <fstream>
#include <sys/stat.h>
#include <stdlib.h>
#include <algorithm>
#include <typeinfo>

#define VEC_MSG ":"
#define INF_MSG "-"
#define DIV_MSG ","
#define END_MSG ";"

#ifndef UTILS_H
#define UTILS_H

namespace utils{
	typedef struct _Dict {
		std::string key;
		std::string value;
	}Dict;
	typedef struct _Ports {
		int n;
		std::string nome;
		Rs232_class* serial;// alocar com new e delete quando o arduino se conectar e desconectar
	}Ports;

	bool path_exist(std::string path);

	void extractKey(std::string &key, size_t const &sepPos, const std::string &line);
	void extractValue(std::string &value, size_t const &sepPos, const std::string &line);
	bool validLine(const std::string &line);
	Ports parseLine(const std::string &line, size_t const lineNo);
	void removeComment(std::string &line);
	std::vector<Ports> get_port_list(const std::string filename);
	std::vector<Dict> get_arduino_list(std::string filename);
	std::vector<Dict> get_msg_list(std::string filename);
	std::vector<std::string> parse_vector_msg(std::string msg, std::string delim = VEC_MSG);
	std::vector<Dict> parse_ard_msg(std::string msg, std::string delim1 = INF_MSG, std::string delim2 = DIV_MSG, std::string delim3 = END_MSG);
	template <typename N>
	std::string numTostr ( N n );
	template <typename V>
	void vector_to_file(std::string filename, std::vector <V> v);
}

#endif