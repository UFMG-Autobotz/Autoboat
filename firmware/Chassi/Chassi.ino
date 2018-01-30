#include <NewPing.h>
#include <Servo.h> 
#include <Wire.h>
#include "WireBotzSlave.h"

// ---- Parâmetros I²C: ----
#define chassi_address 0x1000100  // Endereço do chassi
#define TX_MSG_SIZE 5             // Tamanho do buffer de envio
#define RX_MSG_SIZE 5             // Tamanho do buffer de recebimento
#define I2C_TIMEOUT 300           // Tempo máximo permitido entre duas mensagens

// Buffers
uint8_t RXBuff[RX_MSG_SIZE], TXBuff[TX_MSG_SIZE];

// Tipos de mensagem
enum tipo_TX {power = 1, US_frente_tras, US_esq_dir};
enum tipo_RX {prop = 1, interface};

// Status
bool I2C_ok;
long last_received;

// Pinagem:
enum
{
  // Placa de interface:
  SW1 = 7,
  LED1 = 8,
  LED2 = 12,
  LED3 = 13,
  
  // Propulsão:
  DIR_R = 10,
  PWM_R = 9,
  DIR_L = 6,
  PWM_L = 5,
  
  // Ultrassons:
  U1 = 4,
  U2 = 3,
  U3 = 2,
  U4 = 1,
  
  // Circuito LiPo:
  bat_v = A0,
  bat_i = A1
};

// Motores da propulsão
Servo brushless_L, brushless_R;
Servo servo_L, servo_R;

// Parâmetros de calibração (durações de pulso, em microssegundos)
#define ESC_MIN_0 800   // Valor para o qual o mínimo do ESC foi calibrado 
#define ESC_MIN_1 850   // Valor a partir do qual o ESC de fato começa a girar
#define ESC_MAX   2000  // Valor para o qual o máximo do ESC foi calibrado
#define SERVO_MIN 1000  // Limite inferior do giro do servo (sentido anti-horário)
#define SERVO_MAX 2000  // Limite superior do giro do servo (sentido anti-horário)

// Estado do hardware
uint8_t ang_L, ang_R, vel_L, vel_R;
uint8_t estado_LEDs[3];
bool barco_ligado;

// ---- Ultrassons: ----
// Distância máxima
#define MAX_DIST 255

// Setup
NewPing sonar1(U1,U1,MAX_DIST);
NewPing sonar2(U2,U2,MAX_DIST);
NewPing sonar3(U3,U3,MAX_DIST);
NewPing sonar4(U4,U4,MAX_DIST);
    
// Vetor de leituras
uint16_t distancias[4];

// Funções
void alertaI2C();
void desligaPropulsao();

void setup()
{
  // Configura I²C
  Slave.begin(chassi_address);
  Slave.setRXBuffer(RXBuff, RX_MSG_SIZE);
  Slave.setTXBuffer(TXBuff, TX_MSG_SIZE);
  last_received = millis();
  I2C_ok = false;
  
  // Configura motores da propulsão
  brushless_R.attach(PWM_R,ESC_MIN_1,ESC_MAX);
  brushless_L.attach(PWM_L,ESC_MIN_1,ESC_MAX);
  servo_R.attach(DIR_R,SERVO_MIN,SERVO_MAX);
  servo_L.attach(DIR_L,SERVO_MIN,SERVO_MAX);
  desligaPropulsao();

  // Configura pinos da placa de interface
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  pinMode(SW1,INPUT);
}

void loop()
{
  // Lê o botão da placa de interface
  static bool botao_anterior = false;
  bool botao_atual = digitalRead(SW1);

  if(botao_atual && !botao_anterior)  // Borda de subida
    barco_ligado = !barco_ligado;

  botao_anterior = botao_atual;

  // Lê os ultrassons
  distancias[0] = sonar1.ping_cm();
  distancias[1] = sonar2.ping_cm();
  distancias[2] = sonar3.ping_cm();
  distancias[3] = sonar4.ping_cm();

  // Lê as informações da bateria (OBS: falta definir o intervalo de valores, o medidor ainda não existe)
  int tensao = analogRead(bat_v);
  int corrente = analogRead(bat_i);

  // Atualiza buffers de leitura para que o mestre leia
  switch(abs(Slave.getTXCnt() % 3))  // abs() para lidar com overflow
  {
  case 0:
    TXBuff[0] = US_frente_tras;
    TXBuff[1] = highByte(distancias[0]);
    TXBuff[2] = lowByte(distancias[0]);
    TXBuff[3] = highByte(distancias[1]);
    TXBuff[4] = lowByte(distancias[1]);
    break;
      
  case 1:
    TXBuff[0] = US_esq_dir;
    TXBuff[1] = highByte(distancias[2]);
    TXBuff[2] = lowByte(distancias[2]);
    TXBuff[3] = highByte(distancias[3]);
    TXBuff[4] = lowByte(distancias[3]);
    break;

  case 2:
    TXBuff[0] = power;
    TXBuff[1] = barco_ligado;
    TXBuff[2] = corrente;
    TXBuff[3] = tensao;
  }

  // Verifica se há mensagem do mestre
  if(Slave.newMessage())
  {
    last_received = millis();
    I2C_ok = true;
    
    switch(RXBuff[0]) // Verifica o tipo da mensagem
    {
    case prop:
      vel_L = RXBuff[1];               
      vel_R = RXBuff[3];               
      ang_L = 180 - RXBuff[2]; // O servo esquerdo gira no sentido horário
      ang_R = RXBuff[4];
      break;

    case interface:
      estado_LEDs[0] = RXBuff[1];
      estado_LEDs[1] = RXBuff[2];
      estado_LEDs[2] = RXBuff[3];
    }
  }
  else if(millis() - last_received >= I2C_TIMEOUT)
    I2C_ok = false;

  // Comanda o hardware, caso tudo esteja ok
  if(I2C_ok)
  {
    // Placa de interface
    digitalWrite(LED1,estado_LEDs[0]);
    digitalWrite(LED2,estado_LEDs[1]);
    digitalWrite(LED3,estado_LEDs[2]);
    
    // Propulsão
    if(barco_ligado)
    {
      servo_L.write(ang_L);
      servo_R.write(ang_R);

      // Quando desligado, o brushless fica em ESC_MIN_0 para uma maior margem de segurança
      brushless_L.write(vel_L > 0 ? vel_L : ESC_MIN_0);
      brushless_R.write(vel_R > 0 ? vel_R : ESC_MIN_0);
    }
    else
      desligaPropulsao();
  }
  else
  {
    alertaI2C();
    desligaPropulsao();
  }
}

void alertaI2C()  // A cada segundo, pisca três vezes sim uma não
{
  uint8_t tempo = millis() / 125;
  digitalWrite(LED1,tempo % 2 == 0 && tempo % 8 != 0);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);
}

void desligaPropulsao()
{
  servo_L.write(180);
  servo_R.write(0);
  brushless_L.write(ESC_MIN_0);
  brushless_R.write(ESC_MIN_0);
}

