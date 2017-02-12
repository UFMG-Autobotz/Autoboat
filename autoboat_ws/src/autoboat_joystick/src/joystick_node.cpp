#include <ros/ros.h>
#include <cmath>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>

#include <autoboat_joystick/Prop_msg.h>
#include <autoboat_joystick/Stepper_msg.h>



/*
	TA SUAVE, 
	SO FALTA OLHAR COM O DANIEL QUAL VAI SER O TÓPICO QUE VAI TER QUE LER PRA COMEÇAR A POSTAR NOS TOPICOS DO BARCO
	E OLHAR A QUESTAO DA PROPULSAO

*/

////////////////////////////////////////////////////////////////////////
/////////////////////// Definição dos controles ////////////////////////
////////////////////////////////////////////////////////////////////////

//////////
//BOTOES//
//////////

#define TRIANGULO 0
#define BOINHA 1
#define XIS 2
#define QUADRADO 3
#define L2 4
#define R2 5
#define L1 6
#define R1 7
#define SELECT 8
#define START 9
#define L3 10
#define R3 11

/////////
//EIXOS//
/////////

#define X_ANALOG_ESQUERDO 0
#define Y_ANALOG_ESQUERDO 1
#define Y_ANALOG_DIREITO 2
#define X_ANALOG_DIREITO 3
#define X_SETA 4
#define Y_SETA 5

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


int base_position;
int caracol_position;
bool estado_garra = false;
bool caracol_movimentando = false;


int botao_garra = XIS;
int botao_estica_caracol = R1;
int botao_encolhe_caracol = L1;

int eixo_base = X_ANALOG_ESQUERDO;


ros::Publisher caracol;
ros::Publisher base;
ros::Publisher garra;
ros::Publisher propulsao;

autoboat_joystick::Stepper_msg pub_caracol;

//Função de callback que é chamada todas as vezes que o programa recebe uma alteração do joystick
//nela serão processados os comandos do joystick
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){


	//TRATAMENTO DO X
	//SE A GARRA ESTA ABERTA ELE FECHA
	//SE ESTA FECHADA ELE ABRE
	std_msgs::Bool pub_garra;
	if (joy->buttons[botao_garra] == 1){

		if (estado_garra == false){
			estado_garra = true;
		}else{
			estado_garra = false;
		}
		pub_garra.data = estado_garra;
		garra.publish(pub_garra);	
	}
	

	//TRATAMENTO DO R2
	//MANDAO MOTOR GIRAR TUDO, ATÉ ABRIR TUDO
	//SE O R1 FOR SOLTO ELE PARA


	if(caracol_movimentando == true){
		
		if ( (joy->buttons[botao_encolhe_caracol] == 0) && (pub_caracol.dir.data == -1) ) {

			caracol_movimentando = false;
			pub_caracol.dir.data = 0;
			caracol.publish(pub_caracol);

		}else if (  (joy->buttons[botao_estica_caracol] == 0) && (pub_caracol.dir.data == 1)  ){
			
			caracol_movimentando = false;
			pub_caracol.dir.data = 0;
			caracol.publish(pub_caracol);

		}

	}else{

		if ( (joy->buttons[botao_encolhe_caracol] == 1) && (pub_caracol.dir.data == 0) ) {

			caracol_movimentando = true;
			pub_caracol.setpoint.data = 255;
			pub_caracol.speed.data = 30;
			pub_caracol.dir.data = -1;
			caracol.publish(pub_caracol);

		}else if (  (joy->buttons[botao_estica_caracol] == 1) && (pub_caracol.dir.data == 0)  ){
				
			caracol_movimentando = true;
			pub_caracol.setpoint.data = 255;
			pub_caracol.speed.data = 30;
			pub_caracol.dir.data = 1;
			caracol.publish(pub_caracol);


		}

	}


	//TRATAMENTO DO ANALÓGICO ESQUERDO
	//VE QUAL A DIREÇÃO PELO SINAL DO NUMERO
	//MULTIPLICA UM VALOR MAXIMO DE ROTAÇÃO PELA MAGNITUDE

	autoboat_joystick::Stepper_msg pub_base;

	if ( joy->axes[X_ANALOG_ESQUERDO] != 0  ){
		
		pub_base.setpoint.data = 255;
		pub_base.speed.data =  joy->axes[X_ANALOG_ESQUERDO] * 100  ;
		pub_base.speed.data = log(abs(pub_base.speed.data));
		pub_base.dir.data = joy->axes[X_ANALOG_ESQUERDO]/abs(joy->axes[X_ANALOG_ESQUERDO]);
		base.publish(pub_base);

	}else{
	
		pub_base.dir.data = 0;
		base.publish(pub_base);
	
	}


}

void curent_baseCallback(const std_msgs::Int32::ConstPtr& current_base){

	base_position = current_base->data;

}

void current_caracolCallback(const std_msgs::Int32::ConstPtr& current_caracol){

	caracol_position = current_caracol->data;	

}

int main(int argc, char **argv){
	pub_caracol.dir.data = 0;

	ros::init(argc, argv, "joystick_node");
	ros::NodeHandle nodo;
	
	pub_caracol.dir.data = 0;

	//documento que especifica os tópicos e mensagens
	// https://docs.google.com/spreadsheets/d/1Wz0p-1ZMdkYuoWRdanlzbrur67RxHFoacW9EP3fiRCk/edit

	caracol = nodo.advertise<autoboat_joystick::Stepper_msg>("/autoboat/caracol/caracol_stepper_cmd", 1000);
	base = nodo.advertise<autoboat_joystick::Stepper_msg>("/autoboat/caracol/base_stepper_cmd", 1000);
	garra = nodo.advertise<std_msgs::Bool>("/autoboat/garra/open", 1000);	
	propulsao = nodo.advertise<std_msgs::Bool>("/autoboat/prop", 1000);		

	
	ros::Subscriber joystick = nodo.subscribe("joy", 1000, joyCallback);
	ros::Subscriber current_caracol = nodo.subscribe("/autoboat/caracol/caracol_stepper_current", 1000, current_caracolCallback);
	ros::Subscriber current_base = nodo.subscribe("/autoboat/caracol/base_stepper_current", 1000, curent_baseCallback);

	ros::spin();
}
