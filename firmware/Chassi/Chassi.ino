#include <Wire.h>
#include "WireBotzSlave.h"
#include <NewPing.h>
#include <Servo.h> 

//###########################################
//#######Define parametros do I2C############
//###########################################
#define chassi_address 8
#define TX_MSG_SIZE 5
#define RX_MSG_SIZE 5 

//define os buffers
uint8_t RXBuff[RX_MSG_SIZE];
uint8_t TXBuff[TX_MSG_SIZE];

enum tipo_TX {US_frente_tras = 1, US_esq_dir, interf_LiPO};
enum tipo_RX {prop = 1, interface};

//###########################################
//##############Define a pinagem#############
//###########################################
//pinos da placa de interface
const int SW1 = 7; 
const int LED1 =  8;
const int LED2 =  12;
const int LED3 =  13;

//pinos da propulsao
const int DIRR = 10;
const int PWMR = 9;
const int DIRL = 6;
const int PWML = 5;

//pinagem ultrassons
const int U1 = 4;
const int U2 = 3;
const int U3 = 2;
const int U4 = 1;

// Pinos da placa LiPO
const int bat_v = A0;
const int bat_i = A1;

//###########################################
//####Define parametros da propulsao#########
//###########################################
Servo MotorL, MotorR;
Servo ServoL, ServoR;

//variaveis para armazenar o valor lido do mestre
uint8_t DIRL_valor;
uint8_t DIRR_valor;
uint8_t PWML_valor;
uint8_t PWMR_valor;
uint8_t LED[3];

//###########################################
//####Define parametros do Ultrasson#########
//###########################################

//Distancia maxima
#define MAX_DIST 255

//Setup do sonar
NewPing SONAR1(U1, U1, MAX_DIST);
NewPing SONAR2(U2, U2, MAX_DIST);
NewPing SONAR3(U3, U3, MAX_DIST);
NewPing SONAR4(U4, U4, MAX_DIST);
    
uint16_t ultrassom[4];

void setup() {

    //setup da comunicaço I2C com o mestre
    Slave.begin(chassi_address);
    Slave.setRXBuffer(RXBuff, RX_MSG_SIZE);
    Slave.setTXBuffer(TXBuff, TX_MSG_SIZE);
    
    //setup dos ESCs
    MotorR.attach(PWMR);
    MotorL.attach(PWML);
    ServoR.attach(DIRR);
    ServoL.attach(DIRL);

    MotorR.write(0); 
    MotorL.write(0);
    
    pinMode(LED1,OUTPUT);
    pinMode(LED2,OUTPUT);
    pinMode(LED3,OUTPUT);
    pinMode(SW1, INPUT);
}

void loop() {

    //Le o botão da placa de interface
    bool estado_botao = digitalRead(SW1);

    //le os ultrassons
    ultrassom[0] = SONAR1.ping_cm();
    ultrassom[1] = SONAR2.ping_cm();
    ultrassom[2] = SONAR3.ping_cm();
    ultrassom[3] = SONAR4.ping_cm();

    // Lê as informações da bateria (falta definir o intervalo de valores, o medidor ainda não existe)
    int tensao = analogRead(bat_v);
    int corrente = analogRead(bat_i);

    //Recebe mensagens do mestre
    if( Slave.newMessage() ){

        switch(RXBuff[0])
        {
        case prop:
        
            PWML_valor = RXBuff[1];               
            DIRL_valor = constrain(180 - RXBuff[2], 0, 177);
            PWMR_valor = RXBuff[3];               
            DIRR_valor = constrain(RXBuff[4], 0, 177);

            //Outputa o valor da direcao recebida do mestre
            ServoL.write(DIRL_valor);  
            ServoR.write(DIRR_valor);  
            
            //Outputa o valor da velocidade recebida do mestre        
            MotorR.writeMicroseconds( map(PWMR_valor, 0, 255, 700, 2000) ); 
            MotorL.writeMicroseconds( map(PWML_valor, 0, 255, 700, 2000) );
            break;

        case interface:
            
            LED[0] = RXBuff[1];
            LED[1] = RXBuff[2];
            LED[2] = RXBuff[3];

            //Imprime nos leds o estado enviado pelo mestre
            digitalWrite( LED1, LED[0] );
            digitalWrite( LED2, LED[1] );
            digitalWrite( LED3, LED[2] );
        }
    }

    // Envia mensagens para o mestre
    switch(Slave.getTXCnt() % 3)
    {
    case 0:
        TXBuff[0] = US_frente_tras;
        TXBuff[1] = highByte(ultrassom[0]);
        TXBuff[2] = lowByte (ultrassom[0]);
        TXBuff[3] = highByte(ultrassom[1]);
        TXBuff[4] = lowByte (ultrassom[1]);
        break;
        
    case 1:
        TXBuff[0] = US_esq_dir;
        TXBuff[1] = highByte(ultrassom[2]);
        TXBuff[2] = lowByte (ultrassom[2]);
        TXBuff[3] = highByte(ultrassom[3]);
        TXBuff[4] = lowByte (ultrassom[3]);
        break;

    case 2:
        TXBuff[0] = interf_LiPO;
        TXBuff[1] = estado_botao;
        TXBuff[2] = corrente;
        TXBuff[3] = tensao;
    }
}
