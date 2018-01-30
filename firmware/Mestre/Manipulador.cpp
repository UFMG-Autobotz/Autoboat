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

int leitura_infrav_dentro, leitura_infrav_fora, passo_atual_base, passo_atual_caracol;
bool estado_garra = true;

MotorCC garra(garra_en,garra_1,garra_2);
MotorDePasso base(base_1, base_2, base_3, base_4, 48);
MotorDePasso caracol(caracol_1, caracol_2, caracol_3, caracol_4, 48);

void atualiza_info_manip() 
{ 
  leitura_infrav_dentro = analogRead(infrav_1);
  leitura_infrav_fora = analogRead(infrav_2);
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

void comando_base(int cmd, float vel, int dir)
{
  base.velocidade(vel);
  
  if(dir <= 1 && dir >= -1)
    base.irPara(cmd,dir);
  else
    base.irPara(cmd);
}

void comando_caracol(int cmd, float vel, int dir)
{
  caracol.velocidade(vel);
  
  if(dir <= 1 && dir >= -1)
    caracol.irPara(cmd,dir);
  else
    caracol.irPara(cmd);
}
