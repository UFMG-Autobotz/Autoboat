#include "ros_functions.hpp"

namespace ros_func{
// ---------------- CALLBACK FUNCTIONs -----------------
	namespace sub{
		void base_stepper_cmd(const interface::Stepper_msg& msg){
			Barco_class::set_base_cmd(msg.setpoint.data, msg.speed.data, msg.dir.data);
		}
		
		void caracol_stepper_cmd(const interface::Stepper_msg& msg){
			Barco_class::set_caracol_cmd(msg.setpoint.data, msg.speed.data, msg.dir.data);
		}
		
		void open(const std_msgs::Bool& msg){
			Barco_class::set_garra_state(msg.data);
		}
		
		void prop(const interface::Prop_msg& msg){
			std::vector <float> mot_V(2), mot_A(2);
			mot_V[0] = msg.vel_esq.data;
			mot_V[1] = msg.vel_dir.data;
			mot_A[0] = msg.ang_esq.data;
			mot_A[1] = msg.ang_dir.data;
			Barco_class::set_mot_vel(mot_V);
			Barco_class::set_mot_ang(mot_A);
		}
	}

// ---------------- PUBLISH FUNCTIONs -----------------
	namespace pub{
		
		void LED_laranja(ros::Publisher pubLL){
			static std_msgs::Bool msg;
			msg.data = Barco_class::get_LED_l();
			pubLL.publish(msg);
		}
		
		void LED_azul(ros::Publisher pubLA){
			static std_msgs::Bool msg;
			msg.data = Barco_class::get_LED_a();
			pubLA.publish(msg);
		}
		
		void LED_verde(ros::Publisher pubLV){
			static std_msgs::Bool msg;
			msg.data = Barco_class::get_LED_v();
			pubLV.publish(msg);
		}
		
		void button(ros::Publisher pubBU){
			static std_msgs::Bool msg;
			msg.data = Barco_class::get_but();
			pubBU.publish(msg);
		}
		
		void v_bat(ros::Publisher pubVbat){
			static std_msgs::Float32 msg;
			msg.data = Barco_class::get_v_bat();
			pubVbat.publish(msg);
		}
		
		void i_total(ros::Publisher pubIbat){
			static std_msgs::Float32 msg;
			msg.data = Barco_class::get_i_tot();
			pubIbat.publish(msg);
		}
		
		void IR_out(ros::Publisher pubGO){
			static std_msgs::Bool msg;
			pubGO.publish(msg);
		}
		
		void ardumaster_up(ros::Publisher pubAM){
			static std_msgs::Bool msg;
			msg.data = (Barco_class::check_connection_state(MASTER_ID) != -1);
			pubAM.publish(msg);
		}
		
		void ardumaster_up_since(ros::Publisher pubAMU){
			static std_msgs::Time msg;
			msg.data = ros::Time(Barco_class::check_connection_state(MASTER_ID));
			pubAMU.publish(msg);
		}
		
		void arduboat_up(ros::Publisher pubAB){
			static std_msgs::Bool msg;
			msg.data = (Barco_class::check_connection_state(CHASSI_ID) != -1);
			pubAB.publish(msg);
		}
		
		void arduboat_up_since(ros::Publisher pubABU){
			static std_msgs::Time msg;
			msg.data = ros::Time(Barco_class::check_connection_state(CHASSI_ID));
			pubABU.publish(msg);
		}
		
		void arducol_up(ros::Publisher pubAC){
			static std_msgs::Bool msg;
			msg.data = (Barco_class::check_connection_state(CARACOL_ID) != -1);
			pubAC.publish(msg);
		}
		
		void arducol_up_since(ros::Publisher pubACU){
			static std_msgs::Time msg;
			msg.data = ros::Time(Barco_class::check_connection_state(CARACOL_ID));
			pubACU.publish(msg);
		}
		
		void ultrassons(ros::Publisher pubU){
			static std_msgs::Float32MultiArray msg;
			msg.data = Barco_class::get_ultrassom();
			pubU.publish(msg);
		}
		
		void angulo(ros::Publisher pubA){
			static std_msgs::Float32 msg;
			msg.data = Barco_class::get_ang();
			pubA.publish(msg);
		}
		
		void IR_in(ros::Publisher pubGI){
			static std_msgs::Bool msg;
			msg.data = Barco_class::get_block_state();
			pubGI.publish(msg);
		}
		
		void base_stepper_current(ros::Publisher pubBC){
			static std_msgs::Int32 msg;
			msg.data = Barco_class::get_base().current;
			pubBC.publish(msg);
		}
		
		void caracol_stepper_current(ros::Publisher pubCC){
			static std_msgs::Int32 msg;
			msg.data = Barco_class::get_caracol().current;
			pubCC.publish(msg);
		}
	}
}