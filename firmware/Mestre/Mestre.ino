#include <Wire.h>
#include <WireBotzMaster.h> 

// Endereços I²C
#define chassi_address 8

// Definir o tamanho das mensagens de envio (TX) e recebimento (RX)
#define TX_MSG_SIZE 5
#define RX_MSG_SIZE 5

byte RXbuff[RX_MSG_SIZE], TXbuff[TX_MSG_SIZE];  // Buffers I²C

enum tipo_RX {US_frente_tras = 1, US_esq_dir, interf_LiPO};
enum tipo_TX {prop = 1, interface};

bool ligado = false;  // Estado do Arduíno

// Chassi
enum LED_cor {laranja, azul, verde};
bool estado_led[3], estado_botao; // Placa de interface
uint16_t ultrassom[4];            // Leituras dos ultrassons
int lipo_i, lipo_v;               // Corrente e tensão na bateria

// Manipulador
extern int leitura_sensor_dentro, leitura_sensor_fora, passo_atual_base, passo_atual_caracol;
void atualiza_info_manip();

// IMU
extern float angle_x, angle_y, angle_z;
void setupIMU(), loopIMU();

// Computador embarcado
void envia_sensores(int dentro, int fora),
     envia_steppers(int atual_base, int atual_caracol),
     envia_ultrassons(uint16_t u[4]),
     envia_imu(float ang_x, float ang_y, float ang_z),
     envia_botao(int estado),
     envia_bateria(int i, int v),
     confirma_envio();

void setup()
{       
  Master.begin();  // Inicializa a comunicação do arduíno mestre
  
  setupIMU();
 
  Serial.begin(9600);  
}

void loop()
{
	Master.read(chassi_address, RXbuff, RX_MSG_SIZE); // Recebe informações do escravo

  if(!ligado)                                         // Executa apenas uma vez, para ligar o robô
    if(RXbuff[0] == interf_LiPO && RXbuff[1] == HIGH) // Verifica se o botão foi pressionado
      ligado = true;
    else
      return;
  
  switch(RXbuff[0]) // Confere o tipo da mensagem recebida
  {
  case US_frente_tras:
    ultrassom[0] = word(RXbuff[1], RXbuff[2]);  // Frente
    ultrassom[1] = word(RXbuff[3], RXbuff[4]);  // Trás
    break;    

  case US_esq_dir:
    ultrassom[2] = word(RXbuff[1], RXbuff[2]);  // Esquerda
    ultrassom[3] = word(RXbuff[3], RXbuff[4]);  // Direita
    break;

  case interf_LiPO:
    estado_botao = RXbuff[1];
    lipo_i = RXbuff[2];
    lipo_v = RXbuff[3];
  }

  loopIMU();
  atualiza_info_manip();

  envia_sensores  (leitura_sensor_dentro, leitura_sensor_fora);
  envia_steppers  (passo_atual_base, passo_atual_caracol);
  envia_ultrassons(ultrassom);
  envia_imu       (angle_x, angle_y, angle_z);
  envia_botao     (estado_botao);
  envia_bateria   (lipo_i, lipo_v);

  confirma_envio();
  delay(2);
}

void comando_prop(int vel_esq, int vel_dir, int ang_esq, int ang_dir)
{
  TXbuff[0] = prop;  // Indica que a mensagem se destina aos propulsores
  TXbuff[1] = vel_esq;
  TXbuff[2] = ang_esq;
  TXbuff[3] = vel_dir;
  TXbuff[4] = ang_dir;

  Master.write(chassi_address, TXbuff, TX_MSG_SIZE);
}

void comando_leds(LED_cor cor, bool estado)
{
  estado_led[cor] = estado;

  TXbuff[0] = interface;  // Indica que a mensagem se destina à placa de interface
  TXbuff[1] = estado_led[laranja];
  TXbuff[2] = estado_led[azul];
  TXbuff[3] = estado_led[verde];

  Master.write(chassi_address, TXbuff, TX_MSG_SIZE);
}

