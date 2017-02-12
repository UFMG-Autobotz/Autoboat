#include <Wire.h>
#include "MPU6050.h"
#include "I2Cdev.h"
#include "I2C.h"

float angle_x, angle_y, angle_z;

//Variaveis globais que guardam o ultimo angulo do gyro e os valores filtrados
unsigned long last_read_time;
float         last_x_angle;  // Estes sao os angulos filtrados
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Armazenar os angulos do giroscopio para comparaçao
float         last_gyro_y_angle;
float         last_gyro_z_angle;

//Funcao que seta o ultimo angulo do giroscopio lido e da filtragem
void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) { 
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;   
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}

//  Variaveis usadas para a calibragem dos sensore usados
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
  // Ler os valores brutos dos sensores.
  // Ler os 14 bytes de uma vez contendo aceleraçao, 
  // temperatura e giroscopio.
  // Com as configuraçoes do MPU6050, os valores nao estao
  // filtrados devidamente, logo a funcao retorna o erro dos
  // valores instaveis.
  
  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;
   
  int error = I2C_read (MPU6050_I2C_ADDRESS,MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));

  // Toca os valores altos e baixos
  // Depois disso, os valores registrados sao trocados, 
  // entao a estrutura chamada de x_accel_l nao 
  // longa contem o menor byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
  SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
  SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
  SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
  SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
  SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
  SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

  return error;
}

// O sensor tem que estar em uma superficie bem horizontal 
// enquanto a calibragem estiver acontecendo
void calibrate_sensors() {
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;

  // Discarta os primeiros valores lidos pela IMU
  read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
  
  // Le e faz a media dos 10 primeiros valores lidos pela IMU
  for (int i = 0; i < num_readings; i++) {
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  // Guarda a calibragem bruta dos valores globais
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
}

void setupIMU()
{
  int error;
  uint8_t c;

  //Inicializando comunicação I2C com a IMU
  Wire.begin();

   // Padrao no "power-up":
  //    Gyro de 250 graus/segundo
  //    Aceleracao de 2g
  //    Clock interno de 8Mhz
  //    O dispisitivo esta no "sleep mode".

  error = I2C_read (MPU6050_I2C_ADDRESS,MPU6050_WHO_AM_I, &c, 1);

  // De acordo com o datasheet, o 'sleep' bit
  // deveria ser lido como o '1'. Mas aqui esta lendo como '0'.
  // Esse bit tem que ser limpo, desde que o sensor
  // esteja do sleep mode para power-up. Ate que o
  // bit seja '0'.
  error = I2C_read (MPU6050_I2C_ADDRESS,MPU6050_PWR_MGMT_2, &c, 1);

  // Limpa o 'sleep' bit para comecar as medidas.
  I2C_write_reg (MPU6050_I2C_ADDRESS,MPU6050_PWR_MGMT_1, 0);
  
  //Calibra e inicializa os angulos
  calibrate_sensors();  
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}

void loopIMU()
{
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  
  // Le os valores brutos e retorna o erro.
  error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);
  
  // Get the time of reading for rotation computations
  unsigned long t_now = millis();  

  // A temperatura varia de -40 a +85 graus Celsius.
  // De acordo com o datasheet: 
  //   340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412
   
  dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;

  // Converte os valores do gyro para graus/segundos
  float FS_SEL = 131;
 
  float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
  float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
  float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;
  
  
  // Obtem os valores brutos do acelerometro
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;
  
  // Obtem os valores dos angulos do acelerometro
  float RADIANS_TO_DEGREES = 180/3.14159;
//  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_z = 0;
  
  // Computa os valores filtrados do giroscopio
  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();
  
  // Aplica o filtro complementar nos valores dos angulos dos dois sensores usados - a escolha
  // do alpha foi estimado. 
  float alpha = 0.96;
  angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  angle_z = gyro_angle_z;  //Acelerometro nao da o valor do angulo z
  
  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, gyro_angle_x, gyro_angle_y, gyro_angle_z);
  
  // Manda os dados através da porta serial
  //Angulos filtrados

  // Send the data to the serial port
/*
  Serial.print(F("Angulos filtrados:"));             //Angulos filtrados
  Serial.print(angle_x, 2);
  Serial.print(F(","));
  Serial.print(angle_y, 2);
  Serial.print(F(","));
  Serial.print(angle_z, 2);
  Serial.println(F(""));
*/
}

