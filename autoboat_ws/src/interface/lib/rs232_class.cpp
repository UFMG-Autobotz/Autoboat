#include "rs232_class.hpp"

Rs232_class::Rs232_class(){
	// --------------------- RS-232 Init-------------------------
	bdrate = 9600;
	mode[0] = '8';
	mode[1] = 'N';
	mode[2] = '1';
	mode[3] = 0;
	port_opened = false;
}

Rs232_class::Rs232_class(int p, int r){
	// --------------------- RS-232 Init-------------------------
	bdrate = r;
	mode[0] = '8';
	mode[1] = 'N';
	mode[2] = '1';
	mode[3] = 0;
	port_opened = false;
	open_comport(p);
}

Rs232_class::~Rs232_class(){
	close_comport();
}

void Rs232_class::open_comport(int p){
	cport_nr = p;
	if(RS232_OpenComport(cport_nr, bdrate, mode)){
		std::cout<< "Can not open comport\n";
		exit(0);
	}
	port_opened = true;
	std::cout << "Opened comport " << cport_nr << std::endl;
}

void Rs232_class::close_comport(){
	if(port_opened){
		RS232_CloseComport(cport_nr);
		std::cout << "Closed comport " << cport_nr << std::endl;
	}
	port_opened = false;
}

void Rs232_class::set_baud_rate(int r){
	bdrate = r;
}

template <typename V>
void Rs232_class::send(V var){
	// --------------------- RS-232 Enviar-------------------------

	std::ostringstream convert;
	convert << var;
	std::string send_string = convert.str();
	// for (int i = 0; i < send_string.length(); i += MAX_LENGTH){
	// 	if(send_string.length() - i < MAX_LENGTH)
	// 		RS232_cputs(cport_nr, (send_string.substr(i) + FINAL_CHAR).c_str());
	// 	else if (send_string.length() - i  == MAX_LENGTH){
	// 		RS232_cputs(cport_nr, send_string.substr(i).c_str());
	// 		RS232_cputs(cport_nr, FINAL_CHAR);
	// 	}
	// 	else
	// 		RS232_cputs(cport_nr, send_string.substr(i, MAX_LENGTH).c_str());
	// 	usleep(send_wait);
	// }
	for (std::string::iterator it = send_string.begin(); it != send_string.end(); ++it){
		RS232_SendByte(cport_nr, *it);
		usleep(send_wait);
	}
	RS232_SendByte(cport_nr, FINAL_CHAR);
	std::cout<< "Enviado: " << send_string << " of length "<< send_string.length() << std::endl;
}
template void Rs232_class::send(bool var);
template void Rs232_class::send(int var);
template void Rs232_class::send(float var);
template void Rs232_class::send(double var);
template void Rs232_class::send(char const* var);
template void Rs232_class::send(std::string var);

std::string Rs232_class::receive(){
	// --------------------- RS-232 Ler-------------------------
	int n = RS232_PollComport(cport_nr, buf, 4095);
	std::string ret_data = "";
	if(n > 0){
		buf[n] = 0;   /* always put a "null" at the end of a string! */
		for(int j=0; j < n; j++)
			if(buf[j] < 32 || buf[j] > 126)  /* replace unreadable control-codes by REPLACE_CHAR */
				buf[j] = REPLACE_CHAR;
		ret_data = (char *)buf;
		std::cout<< "Received " << n << " bytes: " << (char *)buf << std::endl;
	}
	else
		std::cout<< "Received " << n << " bytes."<< std::endl;
	return ret_data;
}