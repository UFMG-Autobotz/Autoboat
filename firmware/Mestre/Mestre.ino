/* Firmware do Autoboat - Arduíno Mestre
 * Autores: Daniel Leite Ribeiro e Vittor Faria Pereira
 *          Autobotz UFMG
 */

#include <Wire.h>
#include <WireBotzMaster.h> 

#define MODO_DEBUG true

// ---- Parâmetros I²C: ----
#define chassi_address 0x1000100  // Endereço do chassi
#define TX_MSG_SIZE 5             // Tamanho do buffer de envio
#define RX_MSG_SIZE 5             // Tamanho do buffer de recebimento
#define I2C_DELAY 70              // Atraso entre mensagens consecutivas

// Buffers
uint8_t RXbuff[RX_MSG_SIZE], TXbuff[TX_MSG_SIZE];

// Tipos de mensagem
enum tipo_RX {power = 1, US_frente_tras, US_esq_dir};
enum tipo_TX {prop = 1, interface};

bool I2C_ok;

// Chassi
enum LED_cor {laranja, azul, verde};
bool estado_led[3], ligado; // Placa de interface
uint16_t ultrassons[4];     // Leituras dos ultrassons
int lipo_i, lipo_v;         // Corrente e tensão na bateria

// Manipulador
extern int leitura_infrav_dentro, leitura_infrav_fora, passo_atual_base, passo_atual_caracol;
void atualiza_info_manip();

// IMU
extern float angle_x, angle_y, angle_z;
void setupIMU(), loopIMU();

// Computador embarcado
void prepara_msg_chassi_ok(int ok),
     prepara_msg_infrav(int dentro, int fora),
     prepara_msg_steppers(int atual_base, int atual_caracol),
     prepara_msg_ultrassons(uint16_t u[4]),
     prepara_msg_imu(float ang_x, float ang_y, float ang_z),
     prepara_msg_botao(int estado),
     prepara_msg_bateria(int i, int v),
     envia_msg_serial();
extern bool serial_ok;
extern long last_received;

void setup()
{       
  Master.begin();  // Inicializa a comunicação do arduíno mestre
  
  setupIMU();
 
  Serial.begin(9600);  
}

void loop()
{
	// Recebe informações do chassi
	Master.read(chassi_address, RXbuff, RX_MSG_SIZE);
	I2C_ok = true;
  
  switch(RXbuff[0]) // Confere o tipo da mensagem recebida
  {
  case US_frente_tras:
    ultrassons[0] = word(RXbuff[1], RXbuff[2]);  // Frente
    ultrassons[1] = word(RXbuff[3], RXbuff[4]);  // Trás
    break;    

  case US_esq_dir:
    ultrassons[2] = word(RXbuff[1], RXbuff[2]);  // Esquerda
    ultrassons[3] = word(RXbuff[3], RXbuff[4]);  // Direita
    break;

  case power:
    ligado = RXbuff[1];
    lipo_i = RXbuff[2];
    lipo_v = RXbuff[3];
    break;

  default:
    I2C_ok = false;
  }

  // Recebe informações da IMU e do manipulador
  loopIMU();
  atualiza_info_manip();

  // Confere se o serial está ativo
  if(millis() - last_received >= 500 && !MODO_DEBUG)
    serial_ok = false;

  // Comando para os LEDs, dependendo do estado do barco:

  TXbuff[0] = interface;  // Indica que a mensagem se destina à placa de interface
  
  if(serial_ok)
    if(ligado)    // Se o serial estiver ok e o barco ligado, manda aos LEDs o comando do serial
    {
      TXbuff[1] = estado_led[laranja];
      TXbuff[2] = estado_led[azul];
      TXbuff[3] = estado_led[verde];      
    }
    else          // Se o barco estiver desligado, natal (alterna o LED aceso a cada 150 ms)
    {
      uint8_t tempo = millis()/150 % 3;

      TXbuff[1] = (tempo == 0);
      TXbuff[2] = (tempo == 1);
      TXbuff[3] = (tempo == 2);
    }
  else            // Se o serial tiver caído, mantém apenas o LED verde aceso
  {
    TXbuff[1] = LOW;
    TXbuff[2] = LOW;
    TXbuff[3] = HIGH;    
  }

  // Envia a mensagem ao chassi e confere se retornou 0 (sucesso) ou não (erro)
  I2C_ok = I2C_ok && !Master.write(chassi_address, TXbuff, TX_MSG_SIZE);
  delay(I2C_DELAY);

  // Desliga os propulsores se cair a comunicação com o serial:
  if(!serial_ok)
  {
    TXbuff[0] = prop;  // Indica que a mensagem se destina aos propulsores
    TXbuff[1] = 0;
    TXbuff[2] = 0;
    TXbuff[3] = 0;
    TXbuff[4] = 0;
  
    I2C_ok = I2C_ok && !Master.write(chassi_address, TXbuff, TX_MSG_SIZE);
    delay(I2C_DELAY);
  } 
}

bool chassi_ok()
{
  return ligado && I2C_ok;
}

void comando_prop(float vel_esq, float vel_dir, float ang_esq, float ang_dir)
{
  TXbuff[0] = prop;  // Indica que a mensagem se destina aos propulsores
  TXbuff[1] = vel_esq;
  TXbuff[2] = ang_esq;
  TXbuff[3] = vel_dir;
  TXbuff[4] = ang_dir;

  Master.write(chassi_address, TXbuff, TX_MSG_SIZE);
  delay(I2C_DELAY);
}

void comando_leds(LED_cor cor, bool estado)
{ 
  estado_led[cor] = estado;
}

void responde_serial()
{
  prepara_msg_chassi_ok(I2C_ok);
  prepara_msg_infrav(leitura_infrav_dentro, leitura_infrav_fora);
  prepara_msg_steppers(passo_atual_base, passo_atual_caracol);
  prepara_msg_ultrassons(ultrassons);
  prepara_msg_imu(angle_x, angle_y, angle_z);
  prepara_msg_botao(ligado);
  prepara_msg_bateria(lipo_i, lipo_v);
  envia_msg_serial();
}
