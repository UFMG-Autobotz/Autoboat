#include <Arduino.h>

// Strings de identificação
#define ID_ARDUINO "MAST"  // Identificação deste Arduino
#define Data_msg "Dt_msg"  // Indica que as mensagens seguintes contêm dados
#define ID_msg "Id_msg"    // Indica que a mensagen seguinte contém a identificação do Arduino

// Caracteres de controle
#define END_MSG ";"     // Fim do conjunto de mensagens enviadas
#define DIV_MSG ","     // Separador entre mensagens
#define INF_MSG "-"     // Separador entre o ID da mensagem e seu conteúdo
#define VEC_MSG ":"     // Separador entre diferentes campos de um vetor no conteúdo da mensagem
#define FINAL_CHAR '!'  // Fim do conjunto de mensagens recebidas
#define SPLIT_CHAR '?'  // Obsoleto

// IDs das mensagens
#define M_A     0   // Ângulo (IMU)
#define M_AB    1   // Chassi conectado (antigo arduino boat is up)
#define M_ABU   2   // Obsoleto (arduino boat up since)
#define M_AC    3   // Obsoleto (arduino caracol is up)
#define M_ACU   4   // Obsoleto (arduino caracol up since)
#define M_AM    5   // Obsoleto (arduino mestre is up)
#define M_AMU   6   // Obsoleto (arduino mestre up since)
#define M_ARD   7   // Identificação de Arduíno
#define M_BC    8   // Base current (posição atual da base giratória)
#define M_BU    9   // Button (botão da placa de interface)
#define M_CC   10   // Caracol current (posição atual do caracol)
#define M_G    11   // Garra (abrir e fechar)
#define M_GI   12   // Garra in (sensor infravermelho mais interno)
#define M_GO   13   // Garra out (sensor infravermelho mais externo)
#define M_Ibat 14   // Corrente na bateria
#define M_LA   15   // LED azul
#define M_LL   16   // LED laranja
#define M_LV   17   // LED verde
#define M_P    18   // Propulsão
#define M_SB   19   // Setpoint base (posição desejada para a base giratória)
#define M_SC   20   // Setpoint caracol (posição desejada para o caracol)
#define M_U    21   // Ultrassons
#define M_Vbat 22   // Tensão na bateria

#define buf_len 100 // Tamanho máximo do conjunto de mensagens

char buf[buf_len+1];  // Buffer que armazena o conjunto de mensagens
char* tok[4];         // Ponteiros para cada porção (token) da mensagem
String msg_in;
String msg_out = String(M_ARD) + INF_MSG ID_ARDUINO;
String divmsg = String(DIV_MSG);
bool serial_ok;
long last_received = millis();

enum LED_cor {laranja, azul, verde};

void comando_garra(bool),
     comando_prop(float,float,float,float),
     comando_base(int,float,int),
     comando_caracol(int,float,int),
     comando_leds(LED_cor,bool);
     
void decodifica_msg(), responde_serial();
bool chassi_ok();

void prepara_msg_chassi_ok(int ok)
{
  msg_out += divmsg + M_AB + INF_MSG + ok;
}
     
void prepara_msg_infrav(int dentro, int fora)
{
  //msg_out += divmsg + M_GI + INF_MSG + dentro;  // Atualmente só temos o sensor de fora
  msg_out += divmsg + M_GO + INF_MSG + fora;
}

void prepara_msg_steppers(int atual_base, int atual_caracol)
{
  msg_out += divmsg + M_BC + INF_MSG + atual_base;
  msg_out += divmsg + M_CC + INF_MSG + atual_caracol;
}

void prepara_msg_ultrassons(uint16_t u[])
{
  msg_out += divmsg + M_U + INF_MSG + u[0] + VEC_MSG + u[1] + VEC_MSG + u[2] + VEC_MSG + u[3];
}

void prepara_msg_imu(float ang_x, float ang_y, float ang_z)
{
  // Atualmente o computador só lê o ângulo Z
  
  //msg_out += divmsg + M_A + INF_MSG + ang_x + VEC_MSG + ang_y + VEC_MSG + ang_z;
  msg_out += divmsg + M_A + INF_MSG + ang_z;
}

void prepara_msg_botao(int estado)
{
  msg_out += divmsg + M_BU + INF_MSG + estado;
}

void prepara_msg_bateria(int i, int v)
{
  msg_out += divmsg + M_Ibat + INF_MSG + i;
  msg_out += divmsg + M_Vbat + INF_MSG + v;
}

void envia_msg_serial()
{
  Serial.println(msg_out + END_MSG);    // Envia para o PC
  
  msg_out = String(M_ARD) + INF_MSG ID_ARDUINO; // Reseta buffer com ID do Arduino
}

void serialEvent()
{
  static int i = 0;
  
  while(Serial.available())
  {
    if(i >= buf_len)
      i = 0;
    
    if((buf[i++] = Serial.read()) == FINAL_CHAR)
    {      
      while(Serial.available())
        Serial.read();    // Ignora qualquer terminação que tiver vindo após FINAL_CHAR

      i = 0;
      last_received = millis();
      responde_serial();
      decodifica_msg();
    }
  }
}

void decodifica_msg()
{  
  msg_in = String(strtok(buf,DIV_MSG));
  
  if(msg_in == ID_msg) // Caso o PC esteja perguntando o ID deste Arduino
  {
    serial_ok = false; 
    Serial.print(String(M_ARD) + INF_MSG + ID_ARDUINO + END_MSG);
    return;
  }
  else if(msg_in != Data_msg)  // Se não for ID_msg nem Data_msg, algo errado não está certo
  {
    serial_ok = false;
    return;
  }
  
  // Se chegou até aqui, a primeira mensagem era Data_msg, e a próxima é o ID do Arduino de destino
  msg_in = String(strtok(NULL,DIV_MSG)); // Recebe ID do Arduino de destino
  
  if(msg_in != ID_ARDUINO)   // Se o ID não estiver correto
  {
    serial_ok = false;
    Serial.print(String(M_ARD) + INF_MSG + ID_ARDUINO + END_MSG);   // Envia de volta o ID deste Arduíno
    return;
  }

  // Chegou até aqui: tudo ok. Próximas mensagens são comandos (pares ID-dado) para o hardware
  serial_ok = true;

  if(!chassi_ok())  // Confere se o chassi está conectado antes de comandar o hardware
    return;

  while(msg_in = strtok(NULL,INF_MSG))  // Recebe próximo ID de mensagem, se houver
  {
    switch(msg_in.toInt())  // Converte o ID de string para inteiro
    {
    case M_G:   // Garra
      comando_garra(atoi(strtok(NULL,DIV_MSG END_MSG)));
      break;

    case M_P:   // Propulsores
      tok[0] = strtok(NULL,VEC_MSG);
      tok[1] = strtok(NULL,VEC_MSG);
      tok[2] = strtok(NULL,VEC_MSG);
      tok[3] = strtok(NULL,DIV_MSG END_MSG);

      comando_prop(atof(tok[0]), atof(tok[1]), atof(tok[2]), atof(tok[3]));
      break;

    case M_SB:  // Stepper da base
      tok[0] = strtok(NULL,VEC_MSG);
      tok[1] = strtok(NULL,VEC_MSG);
      tok[2] = strtok(NULL,DIV_MSG END_MSG);

      comando_base(atoi(tok[0]), atof(tok[1]), atoi(tok[2]));
      break;

    case M_SC:  // Stepper do caracol
      tok[0] = strtok(NULL,VEC_MSG);
      tok[1] = strtok(NULL,VEC_MSG);
      tok[2] = strtok(NULL,DIV_MSG END_MSG);
      
      comando_caracol(atoi(tok[0]), atof(tok[1]), atoi(tok[2]));
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
