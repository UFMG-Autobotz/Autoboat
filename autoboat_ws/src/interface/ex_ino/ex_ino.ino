#define ID_ARDUINO "12345"
#define MSG1 1
#define MSG2 2
#define MSG3 3
#define MSGvec 4

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


int i;
char comando[20];
float info1, info2, var1=0, var2=5.3;
int info3, var3=2;
int vec_var[3];
boolean stringCompleta = false;
String msg_type;

// ___________________________________________________________________________
void setup(){
  Serial.begin(9600);
  vec_var[0]=1;
  vec_var[1]=3;
  vec_var[2]=5;
}
void loop(){
  if (stringCompleta){
    stringCompleta = false;
    Serial.print(M_ARD);
    Serial.print(INF_MSG);
    Serial.print(ID_ARDUINO);
    Serial.print(DIV_MSG);
    
    Serial.print(MSG1);
    Serial.print(INF_MSG);
    Serial.print(var1);
    Serial.print(DIV_MSG);
    
    Serial.print(MSG2);
    Serial.print(INF_MSG);
    Serial.print(var2);
    Serial.print(DIV_MSG);
    
    Serial.print(MSG3);
    Serial.print(INF_MSG);
    Serial.print(var3);
    Serial.print(DIV_MSG);
    
    Serial.print(MSGvec);
    Serial.print(INF_MSG);
    Serial.print(vec_var[0]);
    Serial.print(VEC_MSG);
    Serial.print(vec_var[1]);
    Serial.print(VEC_MSG);
    Serial.print(vec_var[2]);
    Serial.println(END_MSG);
  }
  delay(5);
}
void serialEvent(){
  while(Serial.available()){
    comando[i] = Serial.read();
    if (comando[i++] == '\n'){
      while(Serial.available())
        Serial.read();
      i = 0;
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
      info1 = atof(strtok(NULL,DIV_MSG));

      msg_type = String(strtok(NULL,INF_MSG));
      info2 = atof(strtok(NULL,DIV_MSG));

      msg_type = String(strtok(NULL,INF_MSG));
      info3 = atoi(strtok(NULL,END_MSG));

      stringCompleta = true;
    }
  }
}
