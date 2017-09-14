#include <ros/ros.h>
#include <cmath>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>

#include <autoboat_msgs/Prop_msg.h>
#include <autoboat_msgs/Stepper_msg.h>

// -------- JOYSTICK --------

/* Mapeamento dos comandos do joystick de PS2 da Autobotz,
 * da maneira como são decodificados no terminal.
 */

// Botões
#define TRIANGULO 0
#define BOLA      1
#define XIS       2
#define QUADRADO  3
#define L2        4
#define R2        5
#define L1        6
#define R1        7
#define SELECT    8
#define START     9
#define L3        10
#define R3        11

//Eixos
#define X_ANALOG_ESQ 0
#define Y_ANALOG_ESQ 1
#define Y_ANALOG_DIR 2
#define X_ANALOG_DIR 3
#define X_DIGITAL    4
#define Y_DIGITAL    5


// -------- BARCO E ROS --------

#define PROP_MAX 100

const int botao_garra = XIS;
const int botao_estica_caracol = R1;
const int botao_encolhe_caracol = L1;
const int botao_base_horario = R2;
const int botao_base_antihorario = L2;
const int botao_led[] = {QUADRADO, TRIANGULO, BOLA};
const int botao_panico = START;

const int eixo_prop_ang_esq = X_ANALOG_ESQ;
const int eixo_prop_ang_dir = X_ANALOG_DIR;
const int eixo_prop_vel_esq = Y_ANALOG_ESQ;
const int eixo_prop_vel_dir = Y_ANALOG_DIR;

ros::Publisher caracol;
ros::Publisher base;
ros::Publisher garra;
ros::Publisher propulsao;
ros::Publisher led[3];

 /* Contadores para a propulsão, utilizados nos dois callbacks:
  * v: velocidade, a: ângulo
  * e: esquerdo, d: direito
  */
int cnt_ve, cnt_vd, cnt_ae, cnt_ad;


// -------- CALLBACKS --------

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
/* Função chamada todas as vezes que o programa recebe uma alteração do joystick
 * Nela serão processados os comandos do joystick
 */
{
    // --- Variáveis estáticas ---

    static bool estado_garra = false;
    static bool botao_garra_apertado = false;
    static bool botao_led_apertado[] = {false, false, false};
    static bool estado_led[] = {false, false, false};

    static std_msgs::Bool msg_garra, msg_led;
    static autoboat_msgs::Stepper_msg msg_caracol, msg_base;
    static autoboat_msgs::Prop_msg msg_prop;

    // --- Garra ---

    // Só realiza o comando se o botão passa de não apertado para apertado
    if(joy->buttons[botao_garra] == 1 && !botao_garra_apertado)
    {
        botao_garra_apertado = true;

        msg_garra.data = estado_garra = !estado_garra;
        garra.publish(msg_garra);
    }
    else if(joy->buttons[botao_garra] == 0 && botao_garra_apertado)
        botao_garra_apertado = false;

    // --- Caracol ---

    msg_caracol.speed.data = 2;

    // Trata os dois botões como um número de dois bits
    switch(joy->buttons[botao_estica_caracol] << 1 | joy->buttons[botao_encolhe_caracol])
    {
    case 0b01:  // Apenas botão encolher
        msg_caracol.dir.data = -1;
        msg_caracol.setpoint.data = 0;   // Ponto mais encolhido
        break;

    case 0b10:  // Apenas botão esticar
        msg_caracol.dir.data = 1;
        msg_caracol.setpoint.data = 255; // Ponto mais esticado
        break;

    default:    // Ambos soltos (00) ou ambos pressionados (11)
        msg_caracol.dir.data = 0;
    }

    base.publish(msg_caracol);

    // --- Base giratória ---

    msg_base.speed.data = 2;

    /* OBS: Na base, setpoint.data é um valor cíclico de 0 a 47. Portanto, atribui-lo a
     * -1 irá comandar um giro para o sentido anti-horário, mas esse valor jamais será
     * atingido, gerando um giro contúnuo. Idem para 48, porém para o sentido horário.
     */

    // Trata os dois botões como um número de dois bits:
    switch(joy->buttons[botao_base_horario] << 1 | joy->buttons[botao_base_antihorario])
    {
    case 0b01:  // Apenas botão anti-horário
        msg_base.dir.data = -1;
        msg_base.setpoint.data = -1;
        break;

    case 0b10:  // Apenas botão horário
        msg_base.dir.data = 1;
        msg_base.setpoint.data = 48;
        break;

    default:    // Ambos soltos (00) ou ambos pressionados (11)
        msg_base.dir.data = 0;
    }

    base.publish(msg_base);

    // --- LEDs ---

    /*  0: Laranja
     *  1: Azul
     *  2: Verde
     *
     *  Os botões têm comportamento análogo ao da garra
     */

    for(int i = 0; i < 3; i++)
        if(joy->buttons[botao_led[i]] == 1 && !botao_led_apertado[i])
        {
            botao_led_apertado[i] = true;

            msg_led.data = estado_led[i] = !estado_led[i];
            led[i].publish(msg_led);
        }
        else if(joy->buttons[botao_led[i]] == 0 && botao_led_apertado[i])
            botao_led_apertado[i] = false;

    // --- Propulsão ---

    /* Arredondam-se os valores analógicos dos eixos para -1, 0 ou 1.
     *
     * Os analógicos esquerdo e direito comandam os propulsores esquerdo e
     * direito, respectivamente.
     *
     * O eixo vertical dos analógicos comanda a velocidade dos propulsores:
     * para cima aumenta a velocidade, para baixo diminui.
     *
     * O eixo horizontal comanda o ângulo: para a esquerda gira para trás,
     * e para a direita gira para a frente.
     */

    cnt_ve = round(joy->axes[eixo_prop_vel_esq]);
    cnt_vd = round(joy->axes[eixo_prop_vel_dir]);
    cnt_ae = round(joy->axes[eixo_prop_ang_esq]);
    cnt_ad = round(joy->axes[eixo_prop_ang_dir]);

    // --- Pânico (desliga tudo) ---

    if(joy->buttons[botao_panico])
    {

        msg_garra.data = false;
        msg_led.data = false;

        msg_caracol.speed.data = 0;
        msg_caracol.dir.data = 0;

        msg_base.speed.data = 0;
        msg_base.dir.data = 0;

        msg_prop.ang_esq.data = 0;
        msg_prop.ang_dir.data = 0;
        msg_prop.vel_esq.data = 0;
        msg_prop.vel_dir.data = 0;

        caracol.publish(msg_caracol);
        base.publish(msg_base);
        garra.publish(msg_garra);
        propulsao.publish(msg_prop);
        led[0].publish(msg_led);
        led[1].publish(msg_led);
        led[2].publish(msg_led);
    }
}

void timer_callback(const ros::TimerEvent&)
{
    /* Esta função verifica periodicamente (a cada .02 segundos) os valores
     * dos contadores da propulsão, e realiza as ações necessárias.
     *
     * A função implementa o comportamento descrito anteriormente. Se o
     * contador for 1, incrementa-se o valor correspondente (até atingir
     * seu valor máximo). Se for -1, decrementa-se o valor correspondente
     * (até atingir seu valor mínimo). Se for zero, nada é feito.
     *
     * A velocidade dos propulsores é incrementada de 1 em 1 dentro de um
     * intervalo de 0 a PROP_MAX. O ângulo é incrementado de 2 em 2 dentro
     * de um intervalo de 0 a 180.
     */

    static autoboat_msgs::Prop_msg msg_prop;

    if(cnt_ve > 0 && msg_prop.vel_dir.data < PROP_MAX)
        msg_prop.vel_dir.data++;
    else if(cnt_ve < 0 && msg_prop.vel_dir.data > 0)
        msg_prop.vel_dir.data--;

    if(cnt_vd > 0 && msg_prop.vel_esq.data < PROP_MAX)
        msg_prop.vel_esq.data++;
    else if(cnt_vd < 0 && msg_prop.vel_esq.data > 0)
        msg_prop.vel_esq.data--;

    if(cnt_ae > 0 && msg_prop.ang_dir.data < 180)
        msg_prop.ang_dir.data += 2;
    else if(cnt_ae < 0 && msg_prop.ang_dir.data > 0)
        msg_prop.ang_dir.data -= 2;

    if(cnt_ad > 0 && msg_prop.ang_esq.data < 180)
        msg_prop.ang_esq.data += 2;
    else if(cnt_ad < 0 && msg_prop.ang_esq.data > 0)
        msg_prop.ang_esq.data -= 2;

    propulsao.publish(msg_prop);
}


// -------- MAIN --------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick_node");
    ros::NodeHandle nh;

    ros::Rate loopRate(24);

    // Espera autorização para funcionar
    while(!ros::param::param("autoboat/launch/joystick",true))
        loopRate.sleep();

    // Anuncia publishers
    caracol =   nh.advertise<autoboat_msgs::Stepper_msg>("autoboat/caracol/caracol_stepper_cmd", 10);
    base =      nh.advertise<autoboat_msgs::Stepper_msg>("autoboat/caracol/base_stepper_cmd", 10);
    propulsao = nh.advertise<autoboat_msgs::Prop_msg>("autoboat/prop", 10);
    garra =     nh.advertise<std_msgs::Bool>("autoboat/garra/open", 10);
    led[0] =    nh.advertise<std_msgs::Bool>("autoboat/interface/LED_laranja",10);
    led[1] =    nh.advertise<std_msgs::Bool>("autoboat/interface/LED_azul",10);
    led[2] =    nh.advertise<std_msgs::Bool>("autoboat/interface/LED_verde",10);

    // Associa calbacks do joystick e do timer
    ros::Subscriber joy_cb = nh.subscribe("joy", 100, joyCallback);
    ros::Timer joy_timer = nh.createTimer(ros::Duration(.02),timer_callback,false);

    while(ros::ok() && nh.param("autoboat/launch/joystick",true))
    {
        ros::spinOnce();
        loopRate.sleep();
    }
}
