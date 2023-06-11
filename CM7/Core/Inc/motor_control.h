/*
 * motor_contorl.h
 *
 *  Created on: 2023. 3. 6.
 *      Author: NamJunHee
 */
#include <stdio.h>
#ifndef INC_MOTOR_CONTORL_H_
#define INC_MOTOR_CONTORL_H_

#define PI 3.14159265358979323846

#define GearRatio 1/9
#define N1 1
#define N2 9

#define Swiching_f 25000
#define Kt 91*10^(-3)

#endif /* INC_MOTOR_CONTORL_H_ */

typedef struct {

	double start;
	double current;
	double goal;
	double ramp;
	double time;
	int flag;

} Profile_value;

typedef struct {
	double Wc;
	double Kp;
	double Ki;
	double Kd;
	double Ka;

	double output;
	double output_before;
	double output_limit;

	double error;
	double error_before;
	double error_sum;
	double error_sum_limit;
	double error_diff;

	double current;
	double start;
	double goal;
	double input;

} Controller_Var;

typedef struct {

	double angle;
	double velocity;
	double torque;
	double angle_before;

	int Init_angle;

} Motor_State;

typedef struct{

	double torque;
	double theta;
	double velocity;
	int F_flag;

}Impedance ;

typedef struct{
	double Kp;
	double Ki;
	double Kd;
	double output_limit;
	double error_sum_limit;
}Test_PID;

extern Profile_value profile_1;
extern Profile_value profile_2;
extern Profile_value profile_3;

extern Controller_Var vel_ctl_1;
extern Controller_Var vel_ctl_2;
extern Controller_Var vel_ctl_3;

extern Controller_Var pos_ctl_1;
extern Controller_Var pos_ctl_2;
extern Controller_Var pos_ctl_3;

extern Motor_State m_state_1;
extern Motor_State m_state_2;
extern Motor_State m_state_3;

extern Impedance imp_1;
extern Impedance imp_2;
extern Impedance imp_3;

extern Test_PID pos_test_pid;
extern Test_PID vel_test_pid;

void Init_PID();

int Profile(Profile_value* p);
double Position_Controller(Controller_Var* pos, Motor_State* m_state, Profile_value* profile);
double Velocity_Controller(Controller_Var* vel, Controller_Var* pos, Motor_State* m_state, Profile_value* profile, Impedance * imp);

