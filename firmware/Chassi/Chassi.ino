#include <WireBotzSlave.h>
#include <NewPing.h>
#include <string.h>
#include <Servo.h> 
#include <Wire.h>

//###########################################
//#######Define parametros do I2C############
//###########################################
#define slave_address 8
#define TX_MSG_SIZE 6
#define RX_MSG_SIZE 5 

//define os buffers
uint8_t RXBuff[RX_MSG_SIZE];
uint8_t TXBuff[TX_MSG_SIZE];

//###########################################
//##############Define a pinagem#############
//###########################################
//pinos da placa de interface
const int SW1 = 7; 
const int LED1 =  13;
const int LED2 =  12;
const int LED3 =  8;

//pinos da propulsao
const int DIRR = 6;
const int PWMR = 5;
const int DIRL = 10;
const int PWML = 9;

//pinagem ultrassons
const int U1 = 4;
const int U2 = 3;
const int U3 = 2;
const int U4 = 1;


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
uint8_t LEDS;


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
    

int buttonState = 0;        

void setup() {
    
    Serial.begin(9600);
    

    //setup da comunicaço I2C com o mestre
    Slave.begin(slave_address);
    Slave.setRXBuffer(RXBuff, RX_MSG_SIZE);
    Slave.setTXBuffer(TXBuff, TX_MSG_SIZE);
    
    //setup dos ESCs
    MotorR.attach(PWMR);
    MotorL.attach(PWML);
    ServoR.attach(DIRR);
    ServoL.attach(DIRL);
    
    pinMode(LED1,OUTPUT);
    pinMode(LED2,OUTPUT);
    pinMode(LED3,OUTPUT);
    pinMode(SW1, INPUT);
  
}

void loop(){
    
    //Le o boto da placa de interface
    TXBuff[5] = digitalRead(SW1);
    
    //le os ultrassons
    TXBuff[0] = SONAR1.ping_cm();
    TXBuff[1] = SONAR2.ping_cm();
    TXBuff[2] = SONAR3.ping_cm();
    TXBuff[3] = SONAR4.ping_cm();
    
    Serial.print( "S1: " );
    Serial.print( TXBuff[0] );
    Serial.print( "\tS2: " );
    Serial.print( TXBuff[1] );
    Serial.print( "\tS3: " );
    Serial.print( TXBuff[2] );
    Serial.print( "\tS4: " );
    Serial.println( TXBuff[3] );
        
    //Recebe mensagens do mestre
    if( Slave.newMessage() ){
            
        DIRL_valor = RXBuff[0];
        PWML_valor = RXBuff[1];
        
        DIRR_valor = RXBuff[2];
        PWMR_valor = RXBuff[3];
        LEDS = RXBuff[4];
        
        //Imprime nos leds o estado enviado pelo mestre
        digitalWrite( LED1, bitRead(LEDS,0) );
        digitalWrite( LED2, bitRead(LEDS,1) );
        digitalWrite( LED3, bitRead(LEDS,2) );
        
        //Outputa o valor da direcao recebida do mestre
        ServoL.write(DIRL_valor);  
        ServoR.write(DIRR_valor);  
        
        //Outputa o valor da velocidade recebida do mestre        
        MotorR.writeMicroseconds( map(PWMR_valor, 0, 255, 700, 2000) ); 
        MotorL.writeMicroseconds( map(PWML_valor, 0, 255, 700, 2000) );
        
        //Serial.println( LEDS );
        
    }

    //delay(200);


}
