/*
 * motor_control.c
 *
 *  Created on: 2023. 3. 6.
 *      Author: NamJunHee
 */
#include <stdio.h>
#include "main.h"
#include "motor_control.h"

Profile_value profile_1;
Profile_value profile_2;
Profile_value profile_3;

Controller_Var vel_ctl_1;
Controller_Var vel_ctl_2;
Controller_Var vel_ctl_3;

Controller_Var pos_ctl_1;
Controller_Var pos_ctl_2;
Controller_Var pos_ctl_3;

Motor_State m_state_1;
Motor_State m_state_2;
Motor_State m_state_3;

Test_PID pos_test_pid;
Test_PID vel_test_pid;

int Profile(Profile_value *p) {

	if (p->goal != p->current) {
		p->time++;
		if (p->goal > p->current) {
			p->current = p->start + (p->goal - p->start) * p->time / p->ramp;
		} else if (p->goal < p->current) {
			p->current = p->start - (p->start - p->goal) * p->time / p->ramp;
		}
	} else {
		p->start = p->current;
		p->time = 0;
	}

	return p->current;
}

double Position_Controller(Controller_Var *pos, Motor_State *m_state,
		Profile_value *profile) {

	pos->current = m_state->angle;
	pos->input = pos->current;

	pos->error = pos->goal - pos->input;
	pos->error_sum += pos->error;
	pos->error_diff = pos->error - pos->error_before;

	if (pos->error_sum != 0) {
		if (pos->error_sum > pos->error_sum_limit)
			pos->error_sum = pos->error_sum_limit;
		else if (pos->error_sum < -pos->error_sum_limit)
			pos->error_sum = -pos->error_sum_limit;
	}

	pos->output = pos->Kp * pos->error + pos->Ki * pos->error_sum
			+ pos->Kd * pos->error_diff;
	pos->error_before = pos->error;

	//Anti-Windup
	if (pos->output_limit != 0) {
		if (pos->output > pos->output_limit)
			pos->output = pos->output_limit;
		else if (pos->output < -pos->output_limit)
			pos->output = -pos->output_limit;
	}

	return pos->output;
}

double Velocity_Controller(Controller_Var *vel, Controller_Var *pos,
		Motor_State *m_state, Profile_value *profile, Impedance *imp) {

	vel->current = m_state->velocity; //현재 모터 속도
	vel->input = vel->current;

	vel->error = vel->goal - vel->input;
	vel->error_sum += vel->error;
	vel->error_diff = vel->error - vel->error_before;

	if (vel->error_sum_limit != 0) {
		if (vel->error_sum > vel->error_sum_limit)
			vel->error_sum = vel->error_sum_limit;
		else if (vel->error_sum < -vel->error_sum_limit)
			vel->error_sum = -vel->error_sum_limit;
	}

	vel->output = vel->Kp * vel->error + vel->Ki * vel->error_sum
			+ vel->Kd * vel->error_diff;

	vel->error_before = vel->error;

//Anti-Windup
	if (vel->output_limit != 0) {
		if (vel->output > vel->output_limit)
			vel->output = vel->output_limit;
		else if (vel->output < -(vel->output_limit))
			vel->output = -(vel->output_limit);
	}

	return vel->output;
}

void Init_PID() {

//Position_Control

	//0505-2354
	//pos : Kp 0.1, Ki 0, Kd 0, output_limt 2;
	//vel : Kp 1700, Ki 0.5, Kd 0;

	pos_test_pid.Kp = 0.1;
	pos_test_pid.Ki = 0.00;
	pos_test_pid.Kd = 0.00;
	pos_test_pid.error_sum_limit = 1000;
	pos_test_pid.output_limit = 2;

	pos_ctl_1.Kp = 0.1;
	pos_ctl_1.Ki = 0.00;
	pos_ctl_1.Kd = 0.00;

	pos_ctl_2.Kp = 0.1;
	pos_ctl_2.Ki = 0.00;
	pos_ctl_2.Kd = 0.00;

	pos_ctl_3.Kp = 0.1;
	pos_ctl_3.Ki = 0.00;
	pos_ctl_3.Kd = 0.00;

	pos_ctl_1.error_sum_limit = 1000;
	pos_ctl_1.output_limit = 2;

	pos_ctl_2.error_sum_limit = 1000;
	pos_ctl_2.output_limit = 2;

	pos_ctl_3.error_sum_limit = 1000;
	pos_ctl_3.output_limit = 2;

//Velocivy_Control

	vel_test_pid.Kp = 2500;  //2800
	vel_test_pid.Ki = 0.5;   //2
	vel_test_pid.Kd = 0.0;
	vel_test_pid.error_sum_limit = 3000;
	vel_test_pid.output_limit = 9000;


	vel_ctl_1.Kp = 2500;
	vel_ctl_1.Ki = 0.5;
	vel_ctl_1.Kd = 0;

	vel_ctl_2.Kp = 2500;
	vel_ctl_2.Ki = 0.5;
	vel_ctl_2.Kd = 0;

	vel_ctl_3.Kp = 2500;
	vel_ctl_3.Ki = 0.5;
	vel_ctl_3.Kd = 0;

	vel_ctl_1.error_sum_limit = 3000;
	vel_ctl_1.output_limit = 9000;

	vel_ctl_2.error_sum_limit = 3000;
	vel_ctl_2.output_limit = 9000;

	vel_ctl_3.error_sum_limit = 3000;
	vel_ctl_3.output_limit = 9000;

//Profile_value
	profile_1.flag = 0;
	profile_2.flag = 0;
	profile_3.flag = 0;

	profile_1.ramp = 1000;
	profile_2.ramp = 1000;
	profile_3.ramp = 1000;

//Angle_Init
	m_state_1.Init_angle = 225;
	m_state_2.Init_angle = 225;
	m_state_3.Init_angle = 225;

	pos_ctl_1.goal = m_state_1.Init_angle;
	pos_ctl_2.goal = m_state_2.Init_angle;
	pos_ctl_3.goal = m_state_3.Init_angle;

}

