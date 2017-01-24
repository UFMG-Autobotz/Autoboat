#include <Wire.h>
#include <WireBotzMaster.h>

#define chassi_address 8

#define TX_MSG_SIZE 5
#define RX_MSG_SIZE 6

byte RXbuff[RX_MSG_SIZE];
byte TXbuff[TX_MSG_SIZE];

int t = 0;

void setup(){
    Serial.begin(9600);
    Master.begin();
    TXbuff[0] = 0;
    TXbuff[1] = 0;
    TXbuff[2] = 0;
    TXbuff[3] = 0;
}

void loop(){

	Master.read(chassi_address, RXbuff, RX_MSG_SIZE);
	
        //TXbuff[0] = 90; //DIRL
        //TXbuff[1] = 255; //PWML
        
        //TXbuff[2] = 90; //DIRR
        //TXbuff[3] = 255; //PWMR

        if (TXbuff[2] == 180){
            TXbuff[2] = 0;
            TXbuff[0] = 0;
            
            TXbuff[1] = 0;
            TXbuff[3] = 0;
            
            //delay(6000);
        }else{
            TXbuff[2] += 30;
            TXbuff[0] += 30;
            
            TXbuff[1] += 50;
            TXbuff[3] += 50;
            //delay(1000);
        }
        

        //faz um pisca pisca com os leds da placa de interface
        if(t == 3){

            t = 0;
            bitSet(TXbuff[4],0);
            bitSet(TXbuff[4],1);
            bitSet(TXbuff[4],2); 
        
        }else{
            
            if(t == 0){
                
                bitClear(TXbuff[4],0);
                bitClear(TXbuff[4],1);
                bitClear(TXbuff[4],2);
                
            }else if (t == 1){
                
                bitSet(TXbuff[4],0);
                
            }else if(t == 2){
                
                bitSet(TXbuff[4],0);
                bitSet(TXbuff[4],1);
            }
            t++;
            
        }
        delay(500);
        
        
        Master.write(chassi_address, TXbuff, TX_MSG_SIZE);

	Serial.println(RXbuff[5]);
        digitalWrite(9,bitRead(RXbuff[5],0) );
        
	
}
