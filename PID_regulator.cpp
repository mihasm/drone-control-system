#include "PID_regulator.h"

PID_regulator::PID_regulator() {
	
}


void PID_regulator::set_parameters(float Kp, float Ki, float Kd, float bias1) {
	this->Kp=Kp;
	this->Ki=Ki;
	this->Kd=Kd;
	this->bias1=bias1;
}

float PID_regulator::Output(float input, float desired_value1, float dt) {
	actual_value1=input;
	error1=desired_value1-actual_value1;
	integral1=integral_prior1+error1*dt;
	derivative1=(error1-error_prior1)/dt;
	output1=(Kp*error1+Ki*integral1+Kd*derivative1+bias1);
	error_prior1=error1;
	integral_prior1=integral1;
	return output1;
}