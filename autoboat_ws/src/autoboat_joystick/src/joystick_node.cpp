#include <ros/ros.h>
#include <cmath>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>

#include <autoboat_msgs/Prop_msg.h>
#include <autoboat_msgs/Stepper_msg.h>

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

#define PROP_MAX 180

int caracol_position;
bool estado_garra = false;
bool caracol_movimentando = false;

const int botao_garra = XIS;
const int botao_estica_caracol = R1;
const int botao_encolhe_caracol = L1;
const int botao_base_horario = R2;
const int botao_base_antihorario = L2;
const int botao_led[] = {QUADRADO, TRIANGULO, BOINHA};

const int eixo_prop_x_esq = X_ANALOG_ESQUERDO;
const int eixo_prop_x_dir = X_ANALOG_DIREITO;
const int eixo_prop_y_esq = Y_ANALOG_ESQUERDO;
const int eixo_prop_y_dir = Y_ANALOG_DIREITO;

ros::Publisher caracol;
ros::Publisher base;
ros::Publisher garra;
ros::Publisher propulsao;
ros::Publisher led[3];

autoboat_msgs::Stepper_msg pub_caracol;

//Função de callback que é chamada todas as vezes que o programa recebe uma alteração do joystick
//nela serão processados os comandos do joystick
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){


	//TRATAMENTO DO X
	//SE A GARRA ESTA ABERTA ELE FECHA
	//SE ESTA FECHADA ELE ABRE

	static bool botao_garra_apertado = false;
	std_msgs::Bool pub_garra;

	if (joy->buttons[botao_garra] == 1 && !botao_garra_apertado){

		botao_garra_apertado = true;

		if (estado_garra == false){
			estado_garra = true;
		}else{
			estado_garra = false;
		}
		pub_garra.data = estado_garra;
		garra.publish(pub_garra);	
	}
	else if(joy->buttons[botao_garra] == 0 && botao_garra_apertado)
		botao_garra_apertado = false;
	

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
			pub_caracol.setpoint.data = 0;
			pub_caracol.speed.data = 1;
			pub_caracol.dir.data = -1;
			caracol.publish(pub_caracol);

		}else if (  (joy->buttons[botao_estica_caracol] == 1) && (pub_caracol.dir.data == 0)  ){
				
			caracol_movimentando = true;
			pub_caracol.setpoint.data = 255;
			pub_caracol.speed.data = 4;
			pub_caracol.dir.data = 1;
			caracol.publish(pub_caracol);


		}

	}

	// Base giratória:

	autoboat_msgs::Stepper_msg pub_base;
	pub_base.speed.data = 2;

	// Trata os dois botões como um número de dois bits
	switch (joy->buttons[botao_base_horario] << 1 + joy->buttons[botao_base_antihorario])
	{
	case 0b01:  // Apenas anti-horário
		pub_base.dir.data = -1;
		pub_base.setpoint.data = -1;
		break;

	case 0b10:  // Apenas horário
		pub_base.dir.data = 1;
		pub_base.setpoint.data = 48;
		break;

	default:    // Ambos soltos (00) ou ambos pressionados (11)
		pub_base.dir.data = 0;
	}

	base.publish(pub_base);

	/* LEDs
	 *   0: Laranja
	 *   1: Azul
	 *   2: Verde
	 */

	static bool botao_led_apertado[] = {false, false, false};
	static bool estado_led[] = {false, false, false};
	std_msgs::Bool led_msg;

	for(int i = 0; i < 3; i++)
		if(joy->buttons[botao_led[i]] == 1 && !botao_led_apertado[i])
		{
			botao_led_apertado[i] = true;

			led_msg.data = estado_led[i] = !estado_led[i];
			led[i].publish(led_msg);
		}
		else if(joy->buttons[botao_led[i]] == 0 && botao_led_apertado[i])
			botao_led_apertado[i] = false;

	/* Propulsão
	 *   0: Esquerda
	 *   1: Direita
	 */

	autoboat_msgs::Prop_msg msg_prop;

	#define joy_vel(x,y) sqrt(pow((x),2) + pow((y),2))
	#define joy_ang(x,y) std::max(std::min(90-atan2((y),(x))*180/M_PI,0.),180.)

	msg_prop.vel_esq.data = joy_vel( joy->axes[eixo_prop_x_esq], joy->axes[eixo_prop_y_esq]);
	msg_prop.vel_dir.data = joy_vel( joy->axes[eixo_prop_x_dir], joy->axes[eixo_prop_y_dir]);
	msg_prop.ang_esq.data = joy_ang(-joy->axes[eixo_prop_x_esq], joy->axes[eixo_prop_y_esq]);
	msg_prop.ang_dir.data = joy_ang( joy->axes[eixo_prop_x_dir], joy->axes[eixo_prop_y_dir]);

	propulsao.publish(msg_prop);
}

void current_caracolCallback(const std_msgs::Int32::ConstPtr& current_caracol){

	caracol_position = current_caracol->data;	

}

int main(int argc, char **argv){

	ros::init(argc, argv, "joystick_node");
	ros::NodeHandle nodo;

	ros::Rate loopRate(24);

	while(!ros::param::param("/autoboat/launch/joystick",true))
		loopRate.sleep();
	
	pub_caracol.dir.data = 0;

	//documento que especifica os tópicos e mensagens
	// https://docs.google.com/spreadsheets/d/1Wz0p-1ZMdkYuoWRdanlzbrur67RxHFoacW9EP3fiRCk/edit

	caracol = nodo.advertise<autoboat_msgs::Stepper_msg>("/autoboat/caracol/caracol_stepper_cmd", 10);
	base = nodo.advertise<autoboat_msgs::Stepper_msg>("/autoboat/caracol/base_stepper_cmd", 10);
	propulsao = nodo.advertise<autoboat_msgs::Prop_msg>("/autoboat/prop", 10);
	garra = nodo.advertise<std_msgs::Bool>("/autoboat/garra/open", 10);
	led[0] = nodo.advertise<std_msgs::Bool>("/autoboat/interface/LED_laranja",10);
	led[1] = nodo.advertise<std_msgs::Bool>("/autoboat/interface/LED_azul",10);
	led[2] = nodo.advertise<std_msgs::Bool>("/autoboat/interface/LED_verde",10);

	ros::Subscriber joystick = nodo.subscribe("joy", 100, joyCallback);
	ros::Subscriber current_caracol = nodo.subscribe("/autoboat/caracol/caracol_stepper_current", 1, current_caracolCallback);

	while(ros::ok() && nodo.param("/autoboat/launch/joystick",true))
	{
		ros::spinOnce();
		loopRate.sleep();
	}
}
