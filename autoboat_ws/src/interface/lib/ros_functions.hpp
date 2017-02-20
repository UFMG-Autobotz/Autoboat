#include <ros/ros.h>
#include <interface/Stepper_msg.h>
#include <interface/Prop_msg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Time.h>
#include "barco_class.hpp"

#ifndef ROS_FUNC_H
#define ROS_FUNC_H

namespace ros_func{
	namespace sub{
		void base_stepper_cmd(const interface::Stepper_msg& msg);
		void caracol_stepper_cmd(const interface::Stepper_msg& msg);
		void open(const std_msgs::Bool& msg);
		void prop(const interface::Prop_msg& msg);
        void LED_laranja(const std_msgs::Bool& msg);
        void LED_azul(const std_msgs::Bool& msg);
        void LED_verde(const std_msgs::Bool& msg);
	}
	namespace pub{
		void LED_laranja(ros::Publisher pubLL);
		void LED_azul(ros::Publisher pubLA);
		void LED_verde(ros::Publisher pubLV);
		void button(ros::Publisher pubBU);
		void v_bat(ros::Publisher pubVbat);
		void i_total(ros::Publisher pubIbat);
		void IR_out(ros::Publisher pubGO);
		void ardumaster_up(ros::Publisher pubAM);
		void ardumaster_up_since(ros::Publisher pubAMU);
		void arduboat_up(ros::Publisher pubAB);
		void arduboat_up_since(ros::Publisher pubABU);
		void arducol_up(ros::Publisher pubAC);
		void arducol_up_since(ros::Publisher pubACU);
		void ultrassons(ros::Publisher pubU);
		void angulo(ros::Publisher pubA);
		void IR_in(ros::Publisher pubGI);
		void base_stepper_current(ros::Publisher pubBC);
		void caracol_stepper_current(ros::Publisher pubCC);
	}
}

#endif
