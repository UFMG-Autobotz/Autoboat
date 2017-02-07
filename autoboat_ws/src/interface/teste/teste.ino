#define FINAL_CHAR '!'
#define SPLIT_CHAR '?'

#define ID_ARDUINO "TESTE"

#define Data_msg "Dt_msg"
#define Id_msg "Id_msg"

#define VEC_MSG ":"
#define INF_MSG "-"
#define DIV_MSG ","
#define END_MSG ";"

#define M_A 0
#define M_AB 1
#define M_ABU 2
#define M_AC 3
#define M_ACU 4
#define M_AM 5
#define M_AMU 6
#define M_ARD 7
#define M_BC 8
#define M_BU 9
#define M_CC 10
#define M_G 11
#define M_GI 12
#define M_GO 13
#define M_Ibat 14
#define M_LA 15
#define M_LL 16
#define M_LV 17
#define M_P 18
#define M_SB 19
#define M_SC 20
#define M_U 21
#define M_Vbat 22

#define comando_len 150
int i;
char comando[comando_len];
boolean stringCompleta = false;
String msg_type;

float base_speed = 15;
int garra_state = 3, base_dir=5, base_setpoint = 8;
float vec_var[4];
int led_state = LOW;
// ___________________________________________________________________________
void setup(){
  Serial.begin(9600);
  vec_var[0]=1;
  vec_var[1]=3;
  vec_var[2]=5;
  vec_var[3]=7;
  pinMode(13, OUTPUT);
  digitalWrite(13, led_state);
}
void loop(){
  digitalWrite(13, led_state);
  if (stringCompleta){
    stringCompleta = false;
    Serial.print(M_ARD);
    Serial.print(INF_MSG);
    Serial.print(ID_ARDUINO);
    Serial.print(DIV_MSG);

    Serial.print(M_A);
    Serial.print(INF_MSG);
    Serial.print(base_speed);
    Serial.print(DIV_MSG);
    
    Serial.print(M_BC);
    Serial.print(INF_MSG);
    Serial.print(base_setpoint);
    Serial.print(DIV_MSG);
    
    Serial.print(M_BU);
    Serial.print(INF_MSG);
    Serial.print(0);
    Serial.print(DIV_MSG);
    
    Serial.print(M_CC);
    Serial.print(INF_MSG);
    Serial.print(base_dir);
    Serial.print(DIV_MSG);
    
    Serial.print(M_GI);
    Serial.print(INF_MSG);
    Serial.print(0);
    Serial.print(DIV_MSG);
    
    Serial.print(M_Ibat);
    Serial.print(INF_MSG);
    Serial.print(garra_state);
    Serial.print(DIV_MSG);
    
    Serial.print(M_LA);
    Serial.print(INF_MSG);
    Serial.print(0);
    Serial.print(DIV_MSG);
    
    Serial.print(M_LL);
    Serial.print(INF_MSG);
    Serial.print(0);
    Serial.print(DIV_MSG);
    
    Serial.print(M_LV);
    Serial.print(INF_MSG);
    Serial.print(0);
    Serial.print(DIV_MSG);
    
    Serial.print(M_U);
    Serial.print(INF_MSG);
    Serial.print(vec_var[0]);
    Serial.print(VEC_MSG);
    Serial.print(vec_var[1]);
    Serial.print(VEC_MSG);
    Serial.print(vec_var[2]);
    Serial.print(VEC_MSG);
    Serial.print(vec_var[3]);
    Serial.print(DIV_MSG);
    
    Serial.print(M_Vbat);
    Serial.print(INF_MSG);
    Serial.print(0);
    Serial.print(END_MSG);    

  }
  delay(5);
}
void serialEvent(){
  while(Serial.available()){
    if (i >= comando_len)
      i = 0;
    comando[i] = Serial.read();
    if (comando[i++] == FINAL_CHAR){
      while(Serial.available())
        Serial.read();
      i = 0;
      led_state = !led_state;
      msg_type = String(strtok(comando,DIV_MSG));
      if(msg_type == "Id_msg"){
        Serial.print(M_ARD);
        Serial.print(INF_MSG);
        Serial.print(ID_ARDUINO);
        Serial.println(END_MSG);
        break;
      }

      msg_type = String(strtok(NULL,DIV_MSG));
      if(msg_type != ID_ARDUINO){
        Serial.print(M_ARD);
        Serial.print(INF_MSG);
        Serial.print(ID_ARDUINO);
        Serial.println(END_MSG);
        break;
      }

      msg_type = String(strtok(NULL,INF_MSG));
      garra_state = atoi(strtok(NULL,DIV_MSG));

      msg_type = String(strtok(NULL,INF_MSG));
      base_setpoint = atoi(strtok(NULL,VEC_MSG));
      base_speed = atof(strtok(NULL,VEC_MSG));
      base_dir = atoi(strtok(NULL,DIV_MSG));

      msg_type = String(strtok(NULL,INF_MSG));
      vec_var[0]= atof(strtok(NULL,VEC_MSG));
      vec_var[1]= atof(strtok(NULL,VEC_MSG));
      vec_var[2]= atof(strtok(NULL,VEC_MSG));
      vec_var[3]= atof(strtok(NULL,END_MSG));

      stringCompleta = true;
    }
  }

}
