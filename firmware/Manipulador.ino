#include "MotorDePasso.h"
MotorDePasso Caracol;
MotorDePasso Base;
#include <WireBotzSlave.h>
#include <Wire.h>

#define TX_MSG_SIZE_MAN 2
#define RX_MSG_SIZE_MAN 4

uint8_t RXBuff[RX_MSG_SIZE_MAN];
uint8_t TXBuff[TX_MSG_SIZE_MAN];

#define slave_adress 8

static int LeituraSensor;

byte TIPO; byte DIR; byte POS; byte PULSE; 

const int IN1 = 8;   // ponte h da garra
const int IN2 = 9;
const int INPWM = 10;

const int SensorIR = A0;

int i;

void abreGarra()
{
   digitalWrite(IN1,HIGH);
   digitalWrite(IN2,LOW);
   delay(1500);
   digitalWrite(IN1,LOW);
   digitalWrite(IN2,LOW);
}

void fechaGarra()
{
   digitalWrite(IN1,LOW);
   digitalWrite(IN2,HIGH);
   delay(1500);
   digitalWrite(IN1,LOW);
   digitalWrite(IN2,LOW);
}

void setup() {

  
  Slave.begin(slave_adress);

  Slave.setRXBuffer(RXBuff, RX_MSG_SIZE_MAN);
  Slave.setTXBuffer(TXBuff, TX_MSG_SIZE_MAN);

  Base.Pinagem(0,1,2,3);
  Base.PassosPorRevolucao = 48;
  Base.Velocidade = 175;
  
  Caracol.Pinagem(4,5,6,7);
  Caracol.PassosPorRevolucao = 48;
  Caracol.Velocidade = 175;

    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(INPWM,OUTPUT);
    digitalWrite(INPWM,HIGH);

    pinMode(SensorIR,INPUT);


    Serial.begin(9600);
}

void loop() 
{

  digitalWrite(IN1,LOW);
  int i;
  int PASSO_ATUAL = 0;
  
  LeituraSensor = analogRead(SensorIR);
 
  TXBuff[0] = (uint8_t) LeituraSensor;
  TXBuff[1] = (uint8_t) (LeituraSensor >> 8);

  Serial.println(LeituraSensor);

  //Sistema que lê o mestre à ser implementado

  if(Slave.newMessage())
  {
    TIPO = RXBuff[0];
    DIR = RXBuff[1];
    POS = RXBuff[2];
    PULSE = RXBuff[3];   

     if(TIPO == 1) //Motor de Passo da Base
  {
    if(DIR == 1)
    {
      for(i = PASSO_ATUAL;i<=POS;i++)
      {
        Base.Passo(DIR);
        delay(PULSE);
      }
     }
    else if(DIR == -1)
          {
            for(i = PASSO_ATUAL;i<=POS;i--)
            {
              Base.Passo(DIR);
              delay(PULSE);
             }
           }
   }
  else if(TIPO == 2) //Motor de Passos do Caracol
  {
    if(DIR == 1)
    {
      for(i = PASSO_ATUAL;i<POS;i++)
      {
        Caracol.Passo(DIR);
        delay(PULSE);
      }
     }
    else if(DIR == 0)
          {
            for(i = PASSO_ATUAL;i>POS;i--)
            {
              Caracol.Passo(DIR);
              delay(PULSE);
             }
           }
   }
 else if(TIPO == 3) //Motor da Garra
  {
    if(DIR == 0)
      abreGarra();
    else if(DIR == 1)
      fechaGarra();
  }

  } 

  delay(5);
}


