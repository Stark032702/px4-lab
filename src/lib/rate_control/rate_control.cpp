/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RateControl.cpp
 */

#include "rate_control.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setPidGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const Vector3<bool> &saturation_positive,
				      const Vector3<bool> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void RateControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}

void RateControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	_rc_channel_sub.copy(&_rc_channel_values);
	_vehicle_position_sub.copy(&_local_position_values);
	float z_accl = _local_position_values.az;

	pop_editable(_acc_z_values, 80);
	pop_editable(_acc_roll_values, 80);
	pop_editable(_acc_pitch_values, 80);
	pop_editable(_acc_yaw_values, 80);
	push_editable(_acc_z_values, z_accl, 80);
	push_editable(_acc_roll_values, angular_accel(0), 80);
	push_editable(_acc_pitch_values, angular_accel(1), 80);
	push_editable(_acc_yaw_values, angular_accel(2), 80);

	if (stepper % 1 == 0 && !landed){
			update_lambda(angular_accel(0), angular_accel(1), angular_accel(2));
			stepper = 1;

		}
		else{
			stepper += 1;
		}
	if (_rc_channel_values.channels[4] > -1.0f){

		return update_mfc(rate, rate_sp, angular_accel, dt, landed);
	}
	else{
		// angular rates error
		Vector3f rate_error = rate_sp - rate;

		// PID control with feed forward
		const Vector3f p_values = _gain_p.emult(rate_error);
		const Vector3f d_values = _gain_d.emult(angular_accel);
		const Vector3f torque =  p_values + _rate_int - d_values + _gain_ff.emult(rate_sp);

		pidvalues_s pid_values_storage;
		pid_values_storage.timestamp = hrt_absolute_time();
		pid_values_storage.roll_p = p_values(0);
		pid_values_storage.pitch_p = p_values(1);
		pid_values_storage.yaw_p = p_values(2);

		pid_values_storage.roll_d = d_values(0);
		pid_values_storage.pitch_d = d_values(1);
		pid_values_storage.yaw_d = d_values(2);

		pid_values_storage.roll_i = _rate_int(0);
		pid_values_storage.pitch_i = _rate_int(1);
		pid_values_storage.yaw_i = _rate_int(2);

		pid_values_storage.torque[0] = torque(0);
		pid_values_storage.torque[1] = torque(1);
		pid_values_storage.torque[2] = torque(2);

		pid_values_storage.roll_f_hat = 0.0f;
		pid_values_storage.pitch_f_hat = 0.0f;

		pid_values_storage.roll_sp_der = 0.0f;
		pid_values_storage.pitch_sp_der = 0.0f;
		pid_values_storage.yaw_sp_der = 0.0f;

		pid_values_storage.roll_rate_sp = rate_sp(0);
		pid_values_storage.pitch_rate_sp = rate_sp(1);
		pid_values_storage.yaw_rate_sp = rate_sp(2);

		pid_values_storage.roll_rate = rate(0);
		pid_values_storage.pitch_rate = rate(1);
		pid_values_storage.yaw_rate = rate(2);

		pid_values_storage.dt = dt;

		_pid_values_pub.publish(pid_values_storage);


		// update integral only if we are not landed
		if (!landed) {
			updateIntegral(rate_error, dt);
		}

		_last_u = torque;
		return torque;
	}
}

// MFC Functions

void RateControl::setMFCGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D, const float &time_window, const float &SP_der_gain, const float &Lambda, const float &n){

	_mfc_gain_p = P;
	_mfc_gain_i = I;
	_mfc_gain_d = D;
	_gain_sp = SP_der_gain;
	_lambda = Lambda;
	_mfc_n = n;
	_window_size = time_window;

}

Vector3f RateControl::update_mfc(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	const Vector3f p_values = _mfc_gain_p.emult(rate_error);
	const Vector3f d_values = _mfc_gain_d.emult(angular_accel);
	Vector3f torque =  p_values + _rate_int - d_values + _gain_ff.emult(rate_sp);
	//Vector3f constant_vector = Vector3f(1e-34f, 1e-30f, 0.0f);


	pop(_roll_last_u);
	pop(_pitch_last_u);
	pop(_roll_sp_values);
	pop(_pitch_sp_values);
	pop(_roll_rate_values);
	pop(_pitch_rate_values);
	push_pop_time(_time_steps, dt);
	push(_roll_last_u, _last_u(0));
	push(_pitch_last_u, _last_u(1));
	push(_roll_sp_values, rate_sp(0) * 20.0f);
	push(_pitch_sp_values, rate_sp(1) * 20.0f);
	push(_roll_rate_values, rate(0));
	push(_pitch_rate_values, rate(1));
	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);

		//float roll_sp_dd = (angular_accel(0) - _last_accel_values(0)) / dt;
		//float pitch_sp_dd = (angular_accel(1) - _last_accel_values(1)) / dt;
		_sp_double_der(0) = sp_double_dot(_roll_sp_values); //lowpass_filter(dt, 20.0f) * (sp_double_dot(_roll_sp_values) - _sp_double_der(0)); //+= lowpass_filter(dt, 20.0f) * (roll_sp_dd - _sp_double_der(0));
		_sp_double_der(1) = sp_double_dot(_pitch_sp_values); //lowpass_filter(dt, 20.0f) * (sp_double_dot(_pitch_sp_values) - _sp_double_der(1)); //+= lowpass_filter(dt, 20.0f) * (pitch_sp_dd - _sp_double_der(1));
		_f_hat(0) = (_f_hat(0) + lowpass_filter(dt, 5.0f) * math::constrain(F_hat_simpson(_roll_rate_values, _roll_last_u, dt), -.9f * _lambda, .9f *  _lambda)) / 2.0f;   //F_hat_simpson(_roll_rate_values, _roll_last_u);
		_f_hat(1) = (_f_hat(1) + lowpass_filter(dt, 5.0f) * math::constrain(F_hat_simpson(_pitch_rate_values, _pitch_last_u, dt), -.9f * _lambda, .9f * _lambda)) / 2.0f; //F_hat_simpson(_pitch_rate_values, _pitch_last_u);
		torque =  p_values + _rate_int - d_values + _gain_ff.emult(rate_sp);
		_current_u(0) = ((_sp_double_der(0) - _f_hat(0)) / (_lambda)) + torque(0);
		_current_u(1) = ((_sp_double_der(1) - _f_hat(1)) / (_lambda)) + torque(1);
		_current_u(2) = torque(2);
		_last_u = _current_u;
		torque = _current_u;
		//_last_accel_values(0) = angular_accel(0);
		//_last_accel_values(1) = angular_accel(1);

	}

	pidvalues_s pid_values_storage;

	pid_values_storage.timestamp = hrt_absolute_time();
	pid_values_storage.roll_p = p_values(0);
	pid_values_storage.pitch_p = p_values(1);
	pid_values_storage.yaw_p = p_values(2);

	pid_values_storage.roll_d = d_values(0);
	pid_values_storage.pitch_d = d_values(1);
	pid_values_storage.yaw_d = d_values(2);

	pid_values_storage.roll_i = _rate_int(0);
	pid_values_storage.pitch_i = _rate_int(1);
	pid_values_storage.yaw_i = _rate_int(2);

	pid_values_storage.roll_f_hat = _f_hat(0) / (_lambda);
	pid_values_storage.pitch_f_hat = _f_hat(1) / (_lambda);

	pid_values_storage.roll_sp_der = _sp_double_der(0) / ( _lambda);
	pid_values_storage.pitch_sp_der = _sp_double_der(1) / (_lambda);
	pid_values_storage.yaw_sp_der = _sp_double_der(2);

	pid_values_storage.dt = _mfc_dt;

	pid_values_storage.torque[0] = _current_u(0);
	pid_values_storage.torque[1] = _current_u(1);
	pid_values_storage.torque[2] = _current_u(2);

	pid_values_storage.roll_rate_sp = rate_sp(0);
	pid_values_storage.pitch_rate_sp = rate_sp(1);
	pid_values_storage.yaw_rate_sp = rate_sp(2);

	pid_values_storage.roll_rate = rate(0);
	pid_values_storage.pitch_rate = rate(1);
	pid_values_storage.yaw_rate = rate(2);

	_pid_values_pub.publish(pid_values_storage);
	_last_u = torque;

	return torque;
}

float RateControl::F_hat_simpson(float* _f_measurement, float* _last_f_u, float dt){


	float Tot_e[_mfc_n];
	float Tot_o[_mfc_n];
	_mfc_dt = _mfc_n / 400.0f;


	for (int i = 0; i < _mfc_n/2 + 1; i++)
	{
		Tot_e[i+1] = 4.0f*F_hat(_time_steps[2*i + 1], _f_measurement[2*i + 1], _last_f_u[2*i + 1]);
		Tot_o[i] = 2.0f*F_hat(_time_steps[2*i], _f_measurement[2*i], _last_f_u[2*i]);
	}

	Tot_e[0] = 0.0f;
	//Tot_e[_mfc_n/2] = 4.0f*F_hat(_time_steps[1], _f_measurement[1], _last_f_u[1]);
	//Tot_o[0] = F_hat(_time_steps[0], _f_measurement[0], _last_f_u[0]);
	//Tot_o[_mfc_n/2] = F_hat(_time_steps[_mfc_n - 1], _f_measurement[_mfc_n - 1], _last_f_u[_mfc_n - 1]);
	float total = 0.0f;

	for(int k = 0; k < _mfc_n/2 + 1; k++) {
		total += (Tot_e[k] + Tot_o[k])*(dt/3.0f);
	}


	return total;

}

float RateControl::F_hat(float x, float _f1_measurement, float _last_f1_u){


	//_F_hat_calc = (60.0f/(powf(_window_size,5.0f)))*(((powf(_window_size,2.0f) - (6.0f*(_mfc_dt - x))*_window_size + (6.0f* powf((_mfc_dt - x), 2.0f)))*_f1_measurement) -  ((_lambda/2.0f) * powf((_mfc_dt - x), 2.0f) * powf((_window_size - (_mfc_dt - x)), 2.0f) * _last_f1_u));
	_F_hat_calc = (60.0f/(powf(_mfc_dt,5.0f)))*(((powf(_mfc_dt - x,2.0f) - (4.0f*(_mfc_dt - x))*x + (powf((x), 2.0f)))*_f1_measurement) -  ((_lambda/2.0f) * powf((x), 2.0f) * powf((_mfc_dt - x), 2.0f) * _last_f1_u));
	return _F_hat_calc;

}

float RateControl::lowpass_filter(float dt, float cutoff_freq)
{
    float rc = 1.0f / (2.0f * float(M_PI) * cutoff_freq);
    return dt / (dt + rc);
}

float RateControl::sp_double_dot(float* old_setpoints){

	//float final_value = ((powf(_window_size, 2) + 2.0f * _window_size + 1.0f) * old_setpoints[0]) - (2.0f * _window_size + 2.0f*powf(_window_size, 2)) * old_setpoints[1] + (powf(_window_size, 2) * old_setpoints[2]);
	float final_value = (old_setpoints[0] + (2.0f * _window_size + 2.0f*powf(_window_size, 2)) * old_setpoints[1] - (powf(_window_size, 2) * old_setpoints[2]))  /  (powf(_window_size, 2) + 2.0f * _window_size + 1.0f);

	return final_value;


}

void RateControl::pop(float* input){


	for(int i = _mfc_n - 1; i > 0; i--){

		input[i] = input[i - 1];
	}

}

void RateControl::push(float* input, float value){

	input[0] = value;

}

void RateControl::pop_editable(float* input, int n){


	for(int i = 1; i < n; i++){

		input[i - 1] = input[i];
	}

}

void RateControl::push_editable(float* input, float value, int n){

	input[n - 1] = value;

}

float RateControl::sum_editable(float* input, int n){

	float total = 0.0f;
	for(int i = 0; i < n; i++){
		total += input[i];
	}

	return total;
}

void RateControl::push_pop_time(float* input, float value){

	input[0] = input[1] - input[0];
	for(int i = 1; i < _mfc_n - 1; i++){
		input[i] = input[i + 1] - input[i] + input[i - 1];
	}

	input[_mfc_n - 1] = input[_mfc_n - 2] + value;

}

float RateControl::sum(float* input){

	float total = 0.0f;
	for(int i = 0; i < _mfc_n; i++){
		total += input[i];
	}

	return total;
}

void RateControl::scaler_multiplication(const float interval[], float scaler_val, float* output_interval){

	if (scaler_val >= 0.0f){
		output_interval[0] = interval[0] * scaler_val;
		output_interval[1] = interval[1] * scaler_val;
	}
	else {
		output_interval[0] = interval[1] * scaler_val;
		output_interval[1] = interval[0] * scaler_val;
	}
}

float RateControl::max_value(float input_array[]){

	float max_value = input_array[0];
	for (int i = 0; i < 4; i++){
		if (input_array[i] > max_value){
			max_value = input_array[i];
		}
	}
	return max_value;
}

float RateControl::min_value(float input_array[]){

	float min_value = input_array[0];
	for (int i = 0; i < 4; i++){
		if (input_array[i] < min_value){
			min_value = input_array[i];
		}
	}
	return min_value;
}

void RateControl::interval_multiplication(float interval1[], float interval2[], float* outputinterval){

	float multiplication_list[4] = {interval1[0] * interval2[0], interval1[0] * interval2[1], interval1[1] * interval2[0], interval1[1] * interval2[1]};

	outputinterval[0] = max_value(multiplication_list);
	outputinterval[1] = min_value(multiplication_list);

}

void RateControl::interval_addition(float interval1[], float interval2[], float* outputinterval){

	outputinterval[0] = interval1[0] + interval2[0];
	outputinterval[1] = interval1[1] + interval2[1];

}

void RateControl::interval_subtraction(float interval1[], float interval2[], float* outputinterval){

	outputinterval[0] = interval1[0] - interval2[1];
	outputinterval[1] = interval1[1] - interval2[0];
}

void RateControl::interval_intersection(const float interval1[], float interval2[], float* outputinterval){

	float max_value = math::max(interval1[0], interval2[0]);
	float min_value = math::min(interval1[1], interval2[1]);

	if (min_value < max_value){
		outputinterval[0] = outputinterval[0] - 0.001f;  // inflate the interval if there is no solution found
		outputinterval[1] = outputinterval[1] + 0.001f;
	}
	else {
		outputinterval[0] = max_value;
		outputinterval[1] = min_value;
	}
	/* outputinterval[0] = math::max(interval1[0], interval2[0]);
	outputinterval[1] = math::min(interval1[1], interval2[1]); */

}

void RateControl::interval_division(const float interval1[], float interval2[], float* outputinterval){
	// interval1 / intervla2

	if (interval2[0] * interval2[1] > 0.0f){

		float points_array[4] = {interval1[0] / interval2[0], interval1[0] / interval2[1], interval1[1] / interval2[0], interval1[1] / interval2[1]};

		outputinterval[0] = min_value(points_array);
		outputinterval[1] = max_value(points_array);
	}
	/* else {

		if (interval1[0] != 0.0f && interval2[1] != 0.0f){

			outputinterval[0] = -1.0f;
			outputinterval[0] = 1.0f;
		}
		else {

			outputinterval[0] = 0.0f;
			outputinterval[0] = 0.0f;
		}
	} */
}

void RateControl::python_implementation(const float l1[], const float l2[], const float l3[], const float l4[], float actuator_output_array[], float z_accl, float roll_accl, float pitch_accl, float yaw_accl){

	float pwm_acc_z = z_accl / .06f;
	float pwm_acc_roll = roll_accl / _root2over2dct;
	float pwm_acc_pitch = pitch_accl / _root2over2dct;
	float pwm_acc_yaw = yaw_accl / _cm;

	float pwm1_th = (pwm_acc_z + pwm_acc_pitch - pwm_acc_roll + pwm_acc_yaw) / 4.0f;
	float pwm2_th = (pwm_acc_z - pwm_acc_pitch + pwm_acc_roll + pwm_acc_yaw) / 4.0f;
	float pwm3_th = (pwm_acc_z + pwm_acc_pitch + pwm_acc_roll - pwm_acc_yaw) / 4.0f;
	float pwm4_th = (pwm_acc_z - pwm_acc_pitch - pwm_acc_roll - pwm_acc_yaw) / 4.0f;

	float lmda_1 = pwm1_th / actuator_output_array[0];
	float lmda_2 = pwm2_th / actuator_output_array[1];
	float lmda_3 = pwm3_th / actuator_output_array[2];
	float lmda_4 = pwm4_th / actuator_output_array[3];

	_lambda_1[0] = lmda_1;
	_lambda_1[1] = lmda_1;
	_lambda_2[0] = lmda_2;
	_lambda_2[1] = lmda_2;
	_lambda_3[0] = lmda_3;
	_lambda_3[1] = lmda_3;
	_lambda_4[0] = lmda_4;
	_lambda_4[1] = lmda_4;

}

float RateControl::interval_mid_value(float interval[]){

	return (interval[0] + interval[1]) / 2.0f;

}

void RateControl::interval_eq1(const float l1[], const float l2[], const float l3[], const float l4[], float actuator_output_array[], float z_accl){

	/* float constant = _mass * z_accl/(_ct * float(pow(actuator_output_array[0], 2)));
	float mzaccel[2] = {constant, constant};  // constant interval */
	float constant = (z_accl + 9.8f) * (_eq1_const / float(pow(actuator_output_array[0], 2)));

	float mzaccel[2] = {constant, constant};
	float lambda_2_omega[2];
	float lambda_3_omega[2];
	float lambda_4_omega[2];
	scaler_multiplication(l2, float(pow(actuator_output_array[1], 2)), lambda_2_omega);
	scaler_multiplication(l3, float(pow(actuator_output_array[2], 2)), lambda_3_omega);
	scaler_multiplication(l4, float(pow(actuator_output_array[3], 2)), lambda_4_omega);
	float addition_1[2];
	interval_addition(lambda_2_omega, lambda_3_omega, addition_1);
	float addition_2[2];
	interval_addition(addition_1, lambda_4_omega, addition_2);
	float multiplication_1[2];
	scaler_multiplication(addition_2, 1.0f / float(pow(actuator_output_array[0], 2)), multiplication_1);
	float subtraction_1[2];
	interval_subtraction(mzaccel, multiplication_1, subtraction_1);
	//float lmda_value = constant - ((float(pow(actuator_output_array[1], 2)) + float(pow(actuator_output_array[2], 2)) + float(pow(actuator_output_array[3], 2))) / float(pow(actuator_output_array[0], 2)));
	//_lambda_1[0] = lmda_value / -3;
	//_lambda_1[1] = lmda_value / -3;
	interval_intersection(l1, subtraction_1, _lambda_1);

}

void RateControl::interval_eq2(const float l1[], const float l2[], const float l3[], const float l4[], float actuator_output_array[], float roll_accl){

	/* float constant = _jxy * roll_accl/(_root2over2dct * float(pow(actuator_output_array[1], 2)));
	float jxpdot[2] = {constant, constant};  // constant interval */
	float constant = roll_accl * (_eq2_const / float(pow(actuator_output_array[1], 2)));
	float jxpdot[2] = {constant, constant};
	float lambda_1_omega[2];
	float lambda_3_omega[2];
	float lambda_4_omega[2];
	scaler_multiplication(l1, float(pow(actuator_output_array[0], 2)), lambda_1_omega);
	scaler_multiplication(l3, float(pow(actuator_output_array[2], 2)), lambda_3_omega);
	scaler_multiplication(l4, float(pow(actuator_output_array[3], 2)), lambda_4_omega);
	float subtraction_1[2];
	interval_subtraction(lambda_1_omega, lambda_3_omega, subtraction_1);
	float addition_1[2];
	interval_addition(subtraction_1, lambda_4_omega, addition_1);
	float multiplication_1[2];
	scaler_multiplication(addition_1, 1.0f / float(pow(actuator_output_array[1], 2)), multiplication_1);
	float addition_2[2];
	interval_addition(jxpdot, multiplication_1, addition_2);

	interval_intersection(l2, addition_2, _lambda_2);

	/* float lmda_value = constant + ((float(pow(actuator_output_array[0], 2)) - float(pow(actuator_output_array[2], 2)) + float(pow(actuator_output_array[3], 2))) / float(pow(actuator_output_array[1], 2)));
	_lambda_2[0] = lmda_value;
	_lambda_2[1] = lmda_value; */
}

void RateControl::interval_eq3(const float l1[], const float l2[], const float l3[], const float l4[], float actuator_output_array[], float pitch_accl){

	/* float constant = _jxy * pitch_accl/(_root2over2dct * float(pow(actuator_output_array[2], 2)));
	float jyqdot[2] = {constant, constant};  // constant interval */
	float constant = pitch_accl * (_eq3_const / float(pow(actuator_output_array[2], 2)));
	float jyqdot[2] = {constant, constant};
	float lambda_1_omega[2];
	float lambda_2_omega[2];
	float lambda_4_omega[2];
	scaler_multiplication(l1, -1.0f * float(pow(actuator_output_array[0], 2)), lambda_1_omega);
	scaler_multiplication(l2, float(pow(actuator_output_array[1], 2)), lambda_2_omega);
	scaler_multiplication(l4, float(pow(actuator_output_array[3], 2)), lambda_4_omega);
	float addition_1[2];
	interval_addition(lambda_1_omega, lambda_2_omega, addition_1);
	float addition_2[2];
	interval_addition(addition_1, lambda_4_omega, addition_2);
	float multiplication_1[2];
	scaler_multiplication(addition_2, 1.0f / float(pow(actuator_output_array[2], 2)), multiplication_1);
	float addition_3[2];
	interval_addition(jyqdot, multiplication_1, addition_3);

	interval_intersection(l3, addition_3, _lambda_3);

	/* float lmda_value = constant + ((float(pow(actuator_output_array[1], 2)) - float(pow(actuator_output_array[0], 2)) + float(pow(actuator_output_array[3], 2))) / float(pow(actuator_output_array[2], 2)));
	_lambda_3[0] = lmda_value;
	_lambda_3[1] = lmda_value; */

}

void RateControl::interval_eq4(const float l1[], const float l2[], const float l3[], const float l4[], float actuator_output_array[], float yaw_accl){

	/* float constant = _jz * yaw_accl / (_cm * float(pow(actuator_output_array[3], 2)));
	float jzrdot[2] = {constant, constant};  // constant interval */
	float constant = yaw_accl * (_eq4_const / float(pow(actuator_output_array[3], 2)));
	float jzrdot[2] = {constant, constant};
	float lambda_1_omega[2];
	float lambda_2_omega[2];
	float lambda_3_omega[2];
	scaler_multiplication(l1, float(pow(actuator_output_array[0], 2)), lambda_1_omega);
	scaler_multiplication(l2, float(pow(actuator_output_array[1], 2)), lambda_2_omega);
	scaler_multiplication(l3, float(pow(actuator_output_array[2], 2)), lambda_3_omega);
	float addition_1[2];
	interval_addition(lambda_1_omega, lambda_2_omega, addition_1);
	float subtraction_1[2];
	interval_subtraction(addition_1, lambda_3_omega, subtraction_1);
	float multiplication_1[2];
	scaler_multiplication(subtraction_1, 1.0f / float(pow(actuator_output_array[3], 2)), multiplication_1);
	float addition_2[2];
	interval_addition(jzrdot, multiplication_1, addition_2);

	interval_intersection(l4, addition_2, _lambda_4);
	/* float lmda_value = constant + ((float(pow(actuator_output_array[0], 2)) - float(pow(actuator_output_array[1], 2)) - float(pow(actuator_output_array[2], 2))) / float(pow(actuator_output_array[3], 2)));
	_lambda_4[0] = lmda_value;
	_lambda_4[1] = lmda_value; */


}

void RateControl::update_lambda(float roll_accl, float pitch_accl, float yaw_accl){

// 	_vehicle_position_sub.copy(&_local_position_values);
// 	_actuator_output_sub.copy(&_actuator_output_values);

// 	/* float old_lambda_2[2] = {_lambda_2[0], _lambda_2[1]};
// 	float old_lambda_3[2] = {_lambda_3[0], _lambda_3[1]};
// 	float old_lambda_4[2] = {_lambda_4[0], _lambda_4[1]}; */

// 	//float lambda_1_temp[2] = {0.0f};
// 	//float lambda_2_temp[2] = {0.0f};
// 	/* float lambda_3_temp[2] = {0.0f};
// 	float lambda_4_temp[2] = {0.0f}; */


// 	float z_accl = _local_position_values.az;
// 	float actuator_output_array[4] = {float(_actuator_output_values.esc[0].esc_rpm), float(_actuator_output_values.esc[1].esc_rpm), float(_actuator_output_values.esc[2].esc_rpm), float(_actuator_output_values.esc[3].esc_rpm)};
// 	//float actuator_output_array[4] = {sum_editable(_actuator_1, 4) / 4.0f, sum_editable(_actuator_2, 4) / 4.0f, sum_editable(_actuator_3, 4) / 4.0f, sum_editable(_actuator_4, 4) / 4.0f};

// 	/* z_accl = sum_editable(_acc_z_values, 80) / 80.0f;
// 	roll_accl = sum_editable(_acc_roll_values, 80) / 80.0f;
// 	pitch_accl = sum_editable(_acc_pitch_values, 80) / 80.0f;
// 	yaw_accl = sum_editable(_acc_yaw_values, 80) / 80.0f;
//  */
// 	/* float new_z = (0.8f * z_accl) + (_old_z_accl * 0.2f);
// 	float new_roll = (0.8f * roll_accl) + (_old_roll_accl * 0.2f);
// 	float new_pitch = (0.8f * pitch_accl) + (_old_pitch_accl * 0.2f);
// 	float new_yaw = (0.8f * yaw_accl) + (_old_yaw_accl * 0.2f);

// 	_old_z_accl = new_z;
// 	_old_roll_accl = new_roll;
// 	_old_pitch_accl = new_pitch;
// 	_old_yaw_accl = new_yaw; */

// 	float l1[2] = {0.0f};
// 	float l2[2] = {0.0f};
// 	float l3[2] = {0.0f};
// 	float l4[2] = {0.0f};

// 	//python_implementation(l1, l2, l3, l4, actuator_output_array, z_accl, roll_accl, pitch_accl, yaw_accl);
// 	memcpy(l1, &_lambda_1, sizeof(_lambda_1));
// 	memcpy(l2, &_lambda_2, sizeof(_lambda_1));
// 	memcpy(l3, &_lambda_3, sizeof(_lambda_1));
// 	memcpy(l4, &_lambda_4, sizeof(_lambda_1));
// 	interval_eq1(l1, l2, l3, l4, actuator_output_array, z_accl);

// 	memcpy(l1, &_lambda_1, sizeof(_lambda_1));
// 	interval_eq2(l1, l2, l3, l4, actuator_output_array, roll_accl);

// 	memcpy(l2, &_lambda_2, sizeof(_lambda_1));
// 	interval_eq3(l1, l2, l3, l4, actuator_output_array, pitch_accl);

// 	memcpy(l3, &_lambda_3, sizeof(_lambda_1));
// 	interval_eq4(l1, l2, l3, l4, actuator_output_array, yaw_accl);

// 	// publish the data to lambda uorb topic
// 	adaptivecontrollambda_s lambda_values_storage;

// 	lambda_values_storage.timestamp = hrt_absolute_time();
// 	lambda_values_storage.lambda_1 = interval_mid_value(_lambda_1);
// 	lambda_values_storage.lambda_2 = interval_mid_value(_lambda_2);// (0.9f * _lambda_2_old) + (interval_mid_value(_lambda_2) / 10.0f);
// 	lambda_values_storage.lambda_3 = interval_mid_value(_lambda_3); //(0.9f * _lambda_3_old) + (interval_mid_value(_lambda_3) / 10.0f);
// 	lambda_values_storage.lambda_4 = interval_mid_value(_lambda_4); //((1.0f - (1.0f/3.0f)) * _lambda_4_old) + (interval_mid_value(_lambda_4) / 3.0f);

// 	lambda_values_storage.interval_1[0] = _lambda_1[0];
// 	lambda_values_storage.interval_1[1] = _lambda_1[1];
// 	lambda_values_storage.interval_2[0] = _lambda_2[0];
// 	lambda_values_storage.interval_2[1] = _lambda_2[1];
// 	lambda_values_storage.interval_3[0] = _lambda_3[0];
// 	lambda_values_storage.interval_3[1] = _lambda_3[1];
// 	lambda_values_storage.interval_4[0] = _lambda_4[0];
// 	lambda_values_storage.interval_4[1] = _lambda_4[1];


// 	// _lambda_1_old = lambda_values_storage.lambda_1;
// 	// _lambda_2_old = lambda_values_storage.lambda_2;
// 	// _lambda_3_old = lambda_values_storage.lambda_3;
// 	// _lambda_4_old = lambda_values_storage.lambda_4;

// 	/* lambda_values_storage.lambda_1 = new_z; //interval_mid_value(_lambda_1);
// 	lambda_values_storage.lambda_2 = new_roll; //interval_mid_value(_lambda_2);
// 	lambda_values_storage.lambda_3 = new_pitch; //interval_mid_value(_lambda_3);
// 	lambda_values_storage.lambda_4 = new_yaw; //interval_mid_value(_lambda_4); */
// 	_adaptive_control_pub.publish(lambda_values_storage);

}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	float rate_i;
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = 1.0f; //math::max(0.0f, 1.f - i_factor * i_factor);

		// if mfc is running use the mfc gain for i
		if (_rc_channel_values.channels[4] > -1.0f){
			rate_i = _rate_int(i) + i_factor * _mfc_gain_i(i) * rate_error(i) * dt;
		}
		else{
		// Perform the integration using a first order method
			rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;
		}


		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
