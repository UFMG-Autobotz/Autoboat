#include <MotorDePasso.h>
#include <MotorCC.h>

enum  // Pinagem
{
  base_1    = 3,
  base_2    = 4,
  base_3    = 5,
  base_4    = 6,
  caracol_1 = 7,
  caracol_2 = 8,
  caracol_3 = 9,
  caracol_4 = 10,
  garra_1   = 11,
  garra_2   = 12,
  garra_en  = 13,
  infrav_1  = A0,
  infrav_2  = A1
};

int leitura_sensor_dentro, leitura_sensor_fora, passo_atual_base, passo_atual_caracol;
bool estado_garra = true;

MotorDePasso base(base_1, base_2, base_3, base_4, 48),
             caracol(caracol_1, caracol_2, caracol_3, caracol_4, 48);
MotorCC garra(garra_en,garra_1,garra_2);

void atualiza_info_manip() 
{ 
  leitura_sensor_dentro = analogRead(infrav_1);
  leitura_sensor_fora = analogRead(infrav_2);
  passo_atual_base = base.passoAtual();
  passo_atual_caracol = caracol.passoAtual();
}

void comando_garra(bool cmd)
{
  if(cmd && !estado_garra)      // Se a garra deveria estar aberta mas está fechada
    garra.mover(255, 150000);   // Abre a garra
  else if(!cmd && estado_garra) // Se a garra deveria estar fechada mas está aberta
    garra.mover(-255, 150000);  // Fecha a garra

  estado_garra = cmd;
}

void comando_stepper(MotorDePasso& mot, int cmd, float vel, int dir)
{
  int num_passos = cmd - mot.passoAtual();  // Calcula quantos passos deve girar
  mot.velocidade(vel);                      // Define velocidade

  if(dir > 0 && num_passos < 0)             // Se deu negativo mas deveria girar para frente
    num_passos += mot.passosPorRevolucao(); // Toma o equivalente positivo
  else if(dir < 0 && num_passos > 0)        // Se deu positivo mas deveria girar para trás
    num_passos -= mot.passosPorRevolucao(); // Toma o equivalente negativo

  mot.passos(num_passos);                   // Gira o stepper
}

void comando_base(int cmd, float vel, int dir)
{
  comando_stepper(base, cmd, vel, dir);
}

void comando_caracol(int cmd, float vel, int dir)
{
  comando_stepper(caracol, cmd, vel, dir);
}
