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
	if (alternator){
			update_lambda(angular_accel(0), angular_accel(1), angular_accel(2));
			alternator = false;
		}
		else{
			alternator = true;
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

void RateControl::setMFCGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D, const float &Fhat_gain, const float &SP_der_gain, const float &Lambda, const float &n){

	_mfc_gain_p = P;
	_mfc_gain_i = I;
	_mfc_gain_d = D;
	_gain_f_hat = Fhat_gain;
	_gain_sp = SP_der_gain;
	_lambda = Lambda;
	_mfc_n = n;


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
	push(_roll_sp_values, rate_sp(0));
	push(_pitch_sp_values, rate_sp(1));
	push(_roll_rate_values, rate(0));
	push(_pitch_rate_values, rate(1));
	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
		_sp_double_der(0) = F_hat_F(false, true, _roll_sp_values, _roll_last_u) *  _gain_sp;
		_sp_double_der(1) = F_hat_F(false, true, _pitch_sp_values, _pitch_last_u) * _gain_sp;
		_f_hat(0) = F_hat_F(false, false, _roll_rate_values,_roll_last_u) * _gain_f_hat;
		_f_hat(1) = F_hat_F(false, false,  _pitch_rate_values, _pitch_last_u) * _gain_f_hat;
		torque =  p_values + _rate_int - d_values + _gain_ff.emult(rate_sp);
		_current_u(0) = ((_sp_double_der(0) + torque(0) - _f_hat(0)) / _lambda);
		_current_u(1) = ((_sp_double_der(1) + torque(1) - _f_hat(1)) / _lambda);
		_current_u(2) = torque(2);
		_last_u = _current_u;
		torque = _current_u;
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

	pid_values_storage.roll_f_hat = _f_hat(0);
	pid_values_storage.pitch_f_hat = _f_hat(1);

	pid_values_storage.roll_sp_der = _sp_double_der(0);
	pid_values_storage.pitch_sp_der = _sp_double_der(1);
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

float RateControl::F_hat_F(bool y, bool z, float* _f_measurement, float* _last_f_u){


	float Tot_e[_mfc_n];
	float Tot_o[_mfc_n];
	_mfc_dt = _time_steps[_mfc_n - 1];

	for (int i = 0; i < _mfc_n/2 + 1; i++)
	{
		Tot_e[i+1] = 4.0f*F_hat(_time_steps[2*i + 1], y, z, _f_measurement[2*i + 1], _last_f_u[2*i + 1]);
		Tot_o[i] = 2.0f*F_hat(_time_steps[2*i], y, z, _f_measurement[2*i], _last_f_u[2*i]);
	}

	Tot_e[0] = 0.0f;
	Tot_e[_mfc_n/2] = 4.0f*F_hat(_time_steps[1], y, z, _f_measurement[1], _last_f_u[1]);
	Tot_o[0] = F_hat(_time_steps[0], y, z, _f_measurement[0], _last_f_u[0]);
	Tot_o[_mfc_n/2] = F_hat(_time_steps[_mfc_n - 1], y, z, _f_measurement[_mfc_n - 1], _last_f_u[_mfc_n - 1]);
	float total = 0.0f;

	for(int k = 0; k < _mfc_n/2 + 1; k++) {
		total += (Tot_e[k] + Tot_o[k])*(_mfc_dt/3.0f);
	}


	return total;

}

float RateControl::F_hat(float x, bool y, bool z, float _f1_measurement, float _last_f1_u){

	if (y && !z){

		_F_hat_calc = (6/(powf(_mfc_dt,3.0)))*((_mfc_dt - 2*x)*_f1_measurement -  (_lambda * (_mfc_dt - x) * x * _last_f1_u));

		return _F_hat_calc;

	}else if(!y && !z) {

		_F_hat_calc = (60.0f/(powf(_mfc_dt,5.0f)))*(((powf(_mfc_dt,2.0f) - (6.0f*(_mfc_dt - x))*_mfc_dt + (6.0f* powf((_mfc_dt - x), 2.0f)))*_f1_measurement) -  ((_lambda/2.0f) * powf((_mfc_dt - x), 2.0f) * powf((_mfc_dt - (_mfc_dt - x)), 2.0f) * _last_f1_u));

		return _F_hat_calc;

	}
	else if(y && z){

		_F_hat_calc = (6.0f/(powf(_mfc_dt,3.0f)))*((_mfc_dt - 2.0f*x) * _f1_measurement);

		return _F_hat_calc;

	}
	else {

		_F_hat_calc = (60/(powf(_mfc_dt,5.0)))*(((powf(_mfc_dt,2.0) - (6*(_mfc_dt - x))*_mfc_dt + (6* powf((_mfc_dt - x), 2))) * _f1_measurement));

		return _F_hat_calc;
	}
}

void RateControl::pop(float* input){


	for(int i = 1; i < _mfc_n; i++){

		input[i - 1] = input[i];
	}

}

void RateControl::push(float* input, float value){

	input[_mfc_n - 1] = value;

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

void RateControl::scaler_multiplication(float* interval, float scaler_val){

	float interval_copy[2] = {interval[0], interval[1]};

	if (scaler_val >= 0.0f){
		interval[0] = interval_copy[0] * scaler_val;
		interval[1] = interval_copy[1] * scaler_val;
	}
	else {
		interval[0] = interval_copy[1] * scaler_val;
		interval[1] = interval_copy[0] * scaler_val;
	}
}

float RateControl::max_value(float* input_array){

	float max_value = input_array[0];
	for (int i = 0; i < 4; i++){
		if (input_array[i] > max_value){
			max_value = input_array[i];
		}
	}
	return max_value;
}

float RateControl::min_value(float* input_array){

	float min_value = input_array[0];
	for (int i = 0; i < 4; i++){
		if (input_array[i] < min_value){
			min_value = input_array[i];
		}
	}
	return min_value;
}

void RateControl::interval_multiplication(float* interval1, float* interval2, float* outputinterval){

	float multiplication_list[4] = {interval1[0] * interval2[0], interval1[0] * interval2[1], interval1[1] * interval2[0], interval1[1] * interval2[1]};

	outputinterval[0] = max_value(multiplication_list);
	outputinterval[1] = min_value(multiplication_list);

}

void RateControl::interval_addition(float* interval1, float* interval2, float* outputinterval){

	outputinterval[0] = interval1[0] + interval2[0];
	outputinterval[1] = interval1[1] + interval2[1];

}

void RateControl::interval_subtraction(float* interval1, float* interval2, float* outputinterval){

	outputinterval[0] = interval1[0] - interval2[1];
	outputinterval[1] = interval1[1] - interval2[0];
}

void RateControl::interval_union(float* interval1, float* interval2, float* outputinterval){

	outputinterval[0] = math::max(interval1[0], interval2[0]);
	outputinterval[1] = math::min(interval1[1], interval2[1]);
}

float RateControl::interval_mid_value(float* interval){

	return (interval[0] + interval[1]) / 2.0f;

}

void RateControl::update_lambda(float roll_accl, float pitch_accl, float yaw_accl){

	_vehicle_position_sub.copy(&_local_position_values);
	_actuator_output_sub.copy(&_actuator_output_values);

	float* old_lambda_2 = _lambda_2;
	float* old_lambda_3 = _lambda_3;
	float* old_lambda_4 = _lambda_4;

	float lambda_1_temp[2] = {0.0f};
	float lambda_2_temp[2] = {0.0f};
	float lambda_3_temp[2] = {0.0f};
	float lambda_4_temp[2] = {0.0f};


	float z_accl = _local_position_values.az;
	float* actuator_output_array = _actuator_output_values.output;

	float temp_interval[2] = {0.0f};
	float constant;

	scaler_multiplication(old_lambda_2, float(pow(actuator_output_array[1], 2)));
	scaler_multiplication(old_lambda_3, float(pow(actuator_output_array[2], 2)));
	scaler_multiplication(old_lambda_4, float(pow(actuator_output_array[3], 2)));

	// equation 1
	constant = _mass * z_accl/(_ct * float(pow(actuator_output_array[0], 2)));
	float mzaccel[2] = {constant, constant};  // constant interval
	interval_addition(old_lambda_2, old_lambda_3, temp_interval);
	interval_addition(temp_interval, old_lambda_4, lambda_1_temp);

	scaler_multiplication(lambda_1_temp, 1.0f / float(pow(actuator_output_array[0], 2)));

	interval_subtraction(mzaccel, lambda_1_temp, lambda_1_temp);

	interval_union(_lambda_1, lambda_1_temp, _lambda_1);

	float* old_lambda_1 = _lambda_1;  // update lambda values
	scaler_multiplication(old_lambda_1, float(pow(actuator_output_array[0], 2)));

	// euqation 2
	constant = _jxy * roll_accl / (_root2over2dct * float(pow(actuator_output_array[1], 2)));
	float jxpdot[2] = {constant, constant};  // constant interval
	scaler_multiplication(old_lambda_1, -1.0f);
	interval_addition(old_lambda_1, old_lambda_3, temp_interval);
	interval_addition(temp_interval, old_lambda_4, lambda_2_temp);

	scaler_multiplication(lambda_2_temp, 1.0f / float(pow(actuator_output_array[1], 2)));

	interval_addition(jxpdot, lambda_2_temp, lambda_2_temp);

	interval_union(_lambda_2, lambda_2_temp, _lambda_2);

	old_lambda_2 = _lambda_2;
	scaler_multiplication(old_lambda_2, float(pow(actuator_output_array[1], 2)));
	old_lambda_1 = _lambda_1;  // reset lambda 1
	scaler_multiplication(old_lambda_1, float(pow(actuator_output_array[0], 2)));

	// equaiton 3
	constant = _jxy * pitch_accl / (_root2over2dct * float(pow(actuator_output_array[1], 2)));
	jxpdot[0] = constant;
	jxpdot[1] = constant;  // constant interval
	scaler_multiplication(old_lambda_2, -1.0f);
	interval_addition(old_lambda_1, old_lambda_2, temp_interval);
	interval_addition(temp_interval, old_lambda_4, lambda_3_temp);

	scaler_multiplication(lambda_3_temp, 1.0f / float(pow(actuator_output_array[2], 2)));

	interval_addition(jxpdot, lambda_3_temp, lambda_3_temp);

	interval_union(_lambda_3, lambda_3_temp, _lambda_3);

	old_lambda_3 = _lambda_3;
	scaler_multiplication(old_lambda_3, float(pow(actuator_output_array[2], 2)));
	old_lambda_2 = _lambda_2;  // reset lambda 2
	scaler_multiplication(old_lambda_2, float(pow(actuator_output_array[1], 2)));

	//equation 4
	constant = _jz * yaw_accl / (_cm * float(pow(actuator_output_array[3], 2)));
	scaler_multiplication(old_lambda_2, -1.0f);
	interval_addition(old_lambda_1, old_lambda_2, temp_interval);
	interval_addition(temp_interval, old_lambda_3, lambda_4_temp);

	scaler_multiplication(lambda_4_temp, 1.0f / float(pow(actuator_output_array[3], 2)));

	interval_addition(jxpdot, lambda_4_temp, lambda_4_temp);

	interval_union(_lambda_4, lambda_4_temp, _lambda_4);

	// publish the data to lambda uorb topic
	adaptivecontrollambda_s lambda_values_storage;

	lambda_values_storage.lambda_1 = interval_mid_value(_lambda_1);
	lambda_values_storage.lambda_2 = interval_mid_value(_lambda_2);
	lambda_values_storage.lambda_3 = interval_mid_value(_lambda_3);
	lambda_values_storage.lambda_4 = interval_mid_value(_lambda_4);

	_adaptive_control_pub.publish(lambda_values_storage);

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
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

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
