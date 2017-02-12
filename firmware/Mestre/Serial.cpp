#include <Arduino.h>

#define FINAL_CHAR '!'
#define SPLIT_CHAR '?'

#define ID_ARDUINO "TESTE"

#define Data_msg "Dt_msg"
#define ID_msg "Id_msg"

#define VEC_MSG ":"
#define INF_MSG "-"
#define DIV_MSG ","
#define END_MSG ";"

#define M_A     0
#define M_AB    1
#define M_ABU   2
#define M_AC    3
#define M_ACU   4
#define M_AM    5
#define M_AMU   6
#define M_ARD   7
#define M_BC    8
#define M_BU    9
#define M_CC   10
#define M_G    11
#define M_GI   12
#define M_GO   13
#define M_Ibat 14
#define M_LA   15
#define M_LL   16
#define M_LV   17
#define M_P    18
#define M_SB   19
#define M_SC   20
#define M_U    21
#define M_Vbat 22

#define buf_len 150

char buf[buf_len];
String msg_in, msg_out = String(M_ARD) + INF_MSG ID_ARDUINO, divmsg = String(DIV_MSG);

enum LED_cor {laranja, azul, verde};

void comando_garra(bool),
     comando_prop(float,float,float,float),
     comando_base(int,float,int),
     comando_caracol(int,float,int),
     comando_leds(LED_cor,bool);
     
void envia_sensores(int dentro, int fora)
{
//msg_out += divmsg + M_GI + INF_MSG + dentro;  // Atualmente só temos o sensor de fora
  msg_out += divmsg + M_GO + INF_MSG + fora;
}

void envia_steppers(int atual_base, int atual_caracol)
{
  msg_out += divmsg + M_BC + INF_MSG + atual_base;
  msg_out += divmsg + M_CC + INF_MSG + atual_caracol;
}
void envia_ultrassons(uint16_t u[4])
{
  msg_out += divmsg + M_U + INF_MSG + u[0] + VEC_MSG + u[1] + VEC_MSG + u[2] + VEC_MSG + u[3];
}
void envia_imu(float ang_x, float ang_y, float ang_z)
{
//msg_out += divmsg + M_A + INF_MSG + ang_x + VEC_MSG + ang_y + VEC_MSG + ang_z;
  msg_out += divmsg + M_A + INF_MSG + ang_z;  // Atualmente o computador só lê o ângulo Z
}

void envia_botao(int estado)
{
  msg_out += divmsg + M_BU + INF_MSG + estado;
}

void envia_bateria(int i, int v)
{
  msg_out += divmsg + M_Ibat + INF_MSG + i;
  msg_out += divmsg + M_Vbat + INF_MSG + v;
}

void confirma_envio()
{
  Serial.print(msg_out + END_MSG);    // Envia para o PC
  
  msg_out = String(M_ARD) + INF_MSG ID_ARDUINO; // Reseta buffer com ID do Arduino
}

void serialEvent()  // Recebe as mensagens do computador
{
  int i = 0;
  
  // Enche buffer
  while(Serial.available() && i < buf_len && (buf[i] = Serial.read()) != FINAL_CHAR)
    i++;    

  Serial.flush(); // Ignora o que tiver vindo após FINAL_CHAR
    
  msg_in = String(strtok(buf,DIV_MSG));
  
  if(msg_in == ID_msg) // Caso o PC esteja perguntando o ID deste Arduino
  {
    Serial.print(M_ARD);
    Serial.print(INF_MSG);
    Serial.print(ID_ARDUINO);
    Serial.println(END_MSG);
    return;
  }
  else if(msg_in != Data_msg)  // Se não for ID_msg nem Data_msg, algo errado não está certo
    return;

  msg_in = String(strtok(NULL,DIV_MSG)); // Recebe ID do Arduino de destino
  
  if(msg_in != ID_ARDUINO)   // Se o Arduino de destino não for este
  {
    Serial.print(M_ARD);
    Serial.print(INF_MSG);
    Serial.print(ID_ARDUINO);
    Serial.println(END_MSG);
    return;
  }

  while(msg_in = strtok(NULL,INF_MSG))  // Recebe próximo token e verifica se é não-nulo
  {
    switch(msg_in.toInt())
    {
    case M_G:   // Garra
      comando_garra(atoi(strtok(NULL,DIV_MSG END_MSG)));
      break;
    
    case M_P:   // Propulsores
      comando_prop(atof(strtok(NULL,VEC_MSG)), atof(strtok(NULL,VEC_MSG)),
                   atof(strtok(NULL,VEC_MSG)), atof(strtok(NULL,DIV_MSG END_MSG)));
      break;
       
    case M_SB:  // Stepper da base
      comando_base(atoi(strtok(NULL,VEC_MSG)), atof(strtok(NULL,VEC_MSG)), atoi(strtok(NULL,DIV_MSG END_MSG)));
      break;
    
    case M_SC:  // Stepper do caracol
      comando_caracol(atoi(strtok(NULL,VEC_MSG)), atof(strtok(NULL,VEC_MSG)), atoi(strtok(NULL,DIV_MSG END_MSG)));
      break;

    case M_LL:  // LED laranja
      comando_leds(laranja, atoi(strtok(NULL,DIV_MSG END_MSG)));
      break;
      
    case M_LA:  // LED azul
      comando_leds(azul, atoi(strtok(NULL,DIV_MSG END_MSG)));
      break;

    case M_LV:  // LED verde
      comando_leds(verde, atoi(strtok(NULL,DIV_MSG END_MSG)));
    }
  }
}
