#include "utils.hpp"

namespace utils{
	bool path_exist(std::string path){
		static struct stat acm_check;
		if (stat(path.c_str(), &acm_check) == 0)
			return true;
		else
			return false;
	}

	void extractKey(std::string &key, size_t const &sepPos, const std::string &line){
		key = line.substr(0, sepPos);
		if (key.find('\t') != line.npos || key.find(' ') != line.npos)
			key.erase(key.find_first_of("\t "));
	}

	void extractValue(std::string &value, size_t const &sepPos, const std::string &line){
		value = line.substr(sepPos + 1);
		value.erase(0, value.find_first_not_of("\t "));
		value.erase(value.find_last_not_of("\t ") + 1);
	}

	bool validLine(std::string &line){
		std::string temp = line;
		temp.erase(0, temp.find_first_not_of("\t "));
		if (temp[0] == '=')
			return false;
		for (size_t i = temp.find('=') + 1; i < temp.length(); i++)
			if (temp[i] != ' ')
				return true;
		return false;
	}

	Dict parseLine(std::string &line, size_t const lineNo){
		if (line.find('=') == line.npos){
			std::cout << " '=' not found in line " << lineNo << std::endl;
			exit(EXIT_FAILURE);
		}
		if (!validLine(line)){
			std::cout << "Bad format in line " << lineNo << std::endl;
			exit(EXIT_FAILURE);
		}

		line.erase(0, line.find_first_not_of("\t "));
		size_t sepPos = line.find('=');

		std::string key, value;
		extractKey(key, sepPos, line);
		extractValue(value, sepPos, line);
		Dict d;
		d.key = key;
		d.value = value;
		return d;
	}

	void removeComment(std::string &line){
		if (line.find('#') != line.npos)
			line.erase(line.find('#'));
	}

	std::vector<Ports> get_port_list(std::string filename){
		std::ifstream file;
		file.open(filename.c_str());
		if (!file){
			std::cout << "Config File " + filename + " not found!" << std::endl;
			exit(EXIT_FAILURE);
		}
		std::string line;
		size_t cnt_line = 0;
		std::vector<Ports> p_vec;
		while (std::getline(file, line)){
			cnt_line++;
			std::string temp = line;

			if (temp.empty())
				continue;

			removeComment(temp);
			if (temp.find_first_not_of(' ') == temp.npos)
				continue;
			Dict d = parseLine(temp, cnt_line);
			int n = atoi(d.value.c_str());
			Ports p;
			p.n = n;
			p.nome = d.key;
			p_vec.push_back(p);
		}

		file.close();
		return p_vec;
	}

	std::vector<Dict> get_arduino_list(std::string filename){
		std::ifstream file;
		file.open(filename.c_str());
		if (!file){
			std::cout << "Config File " + filename + " not found!" << std::endl;
			exit(EXIT_FAILURE);
		}
		std::string line;
		size_t cnt_line = 0;
		std::vector<Dict> d_vec;
		while (std::getline(file, line)){
			cnt_line++;
			std::string temp = line;

			if (temp.empty())
				continue;

			removeComment(temp);
			if (temp.find_first_not_of(' ') == temp.npos)
				continue;

			d_vec.push_back(parseLine(temp, cnt_line));
		}

		file.close();
		return d_vec;
	}

	std::vector<Dict> get_msg_list(std::string filename){
		std::ifstream file;
		file.open(filename.c_str());
		if (!file){
			std::cout << "Config File " + filename + " not found!" << std::endl;
			exit(EXIT_FAILURE);
		}
		std::string line;
		size_t cnt_line = 0;
		std::vector<Dict> d_vec;
		while (std::getline(file, line)){
			cnt_line++;
			std::string temp = line;

			if (temp.empty())
				continue;

			removeComment(temp);
			if (temp.find_first_not_of(' ') == temp.npos)
				continue;

			d_vec.push_back(parseLine(temp, cnt_line));
		}

		file.close();
		return d_vec;
	}

	std::vector<std::string> parse_vector_msg(std::string msg, std::string delim){
		int start = 0, end = 0;
		std::vector<std::string> s_vec;
		while (end != std::string::npos){
			end = msg.find(delim, start);
			s_vec.push_back(msg.substr(start, end - start));
			start = end + delim.length();
		}
		return s_vec;
	}

	std::vector<Dict> parse_ard_msg(std::string msg, std::string delim1, std::string delim2, std::string delim3){
		int start = 0, end = 0, end2 = msg.find(delim3, 0);
		Dict d;
		std::vector<Dict> d_vec;
		// std::cout << "Parsing message " << msg << std::endl;
		if(end2 != std::string::npos)
			while (end != std::string::npos && end < end2){
				end = msg.find(delim1, start);
				d.key = msg.substr(start, std::min(end,end2) - start);
				start = end + delim1.length();

				end = msg.find(delim2, start);
				if(end == std::string::npos)
					end = end2;
				d.value = msg.substr(start, std::min(end,end2) - start);
				d_vec.push_back(d);
				start = end + delim2.length();
			}
		return d_vec;
	}

	template <typename N>
	std::string numTostr ( N n ){
		std::ostringstream ss;
		ss << n;
		return ss.str();
	}
	template std::string numTostr(int var);
	template std::string numTostr(float var);
	template std::string numTostr(double var);
	template std::string numTostr(time_t var);

	template <>
	void vector_to_file(std::string filename, std::vector <Dict> v){
		std::ofstream f(filename.c_str(), std::ios::app);
		for(int i = 0; i < v.size(); ++i)
			std::cout << v[i].key << " - " << v[i].value << '\n';
		for(int i = 0; i < v.size(); ++i)
				f << v[i].key << " - " << v[i].value << '\n';
		f << '\n';
		f.close();
	}

	template <typename V>
	void vector_to_file(std::string filename, std::vector <V> v){
		std::ofstream f(filename.c_str(), std::ios::app);
		for(int i = 0; i < v.size(); ++i)
				f << v[i] << '\n';
		f << '\n';
		f.close();
	}
	template void vector_to_file(std::string filename, std::vector <std::string> v);
	template void vector_to_file(std::string filename, std::vector <int> v);
	template void vector_to_file(std::string filename, std::vector <float> v);
	template void vector_to_file(std::string filename, std::vector <double> v);
	template void vector_to_file(std::string filename, std::vector <bool> v);
	template void vector_to_file(std::string filename, std::vector <long> v);
	template void vector_to_file(std::string filename, std::vector <char const*> v);
}