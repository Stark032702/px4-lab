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
 * @file rate_control.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <mathlib/mathlib.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/pidvalues.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/adaptivecontrollambda.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>


class RateControl
{
public:
	RateControl() = default;
	~RateControl() = default;

	/**
	 * Set the rate control PID gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setPidGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector3<bool> &saturation_positive,
				 const matrix::Vector3<bool> &saturation_negative);

	/**
	 * Set individual saturation flags
	 * @param axis 0 roll, 1 pitch, 2 yaw
	 * @param is_saturated value to update the flag with
	 */
	void setPositiveSaturationFlag(size_t axis, bool is_saturated);
	void setNegativeSaturationFlag(size_t axis, bool is_saturated);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed);
	// MFC Functions
	matrix::Vector3f update_mfc(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed);

	float F_hat(float x, bool y, bool z, float _f1_measurement, float _last_f1_u);

    	float F_hat_F(bool y, bool z, float* _f_measurement, float* _last_f_u);

	void setMFCGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D, const float &Fhat_gain, const float &SP_der_gain, const float &Lambda, const float &n);

	void pop(float* input);
	void push(float* input, float value);
	void push_pop_time(float* input, float value);
	float sum(float* input);
	void scaler_multiplication(float* interval, float scaler_val);
	void interval_multiplication(float* interval1, float* interval2, float* outputinterval);
	void interval_addition(float* interval1, float* interval2, float* outputinterval);
	void interval_subtraction(float* interval1, float* interval2, float* outputinterval);
	void interval_union(float* interval1, float* interval2, float* outputinterval);
	float max_value(float* input_array);
	float min_value(float* input_array);
	void update_lambda(float roll_accl, float pitch_accl, float yaw_accl);
	float interval_mid_value(float* interval);

	uORB::Subscription _rc_channel_sub{ORB_ID(rc_channels)};
	uORB::Subscription _vehicle_position_sub{ORB_ID(vehicle_local_position)};	// vehicle z acceleration data for adaptive control
	uORB::Subscription _actuator_output_sub{ORB_ID(actuator_outputs)};	// subscribe to the pwm values for motors

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral() { _rate_int.zero(); }

	/**
	 * Set the integral term to 0 for specific axes
	 * @param  axis roll 0 / pitch 1 / yaw 2
	 * @see _rate_int
	 */
	void resetIntegral(size_t axis)
	{
		if (axis < 3) {
			_rate_int(axis) = 0.f;
		}
	}

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);

private:
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	matrix::Vector3f _rate_int; ///< integral term of the rate controller

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;

	uORB::Publication<pidvalues_s>     		_pid_values_pub{ORB_ID(pid_values)};
	uORB::Publication<adaptivecontrollambda_s>	_adaptive_control_pub{ORB_ID(adaptive_lambda)}; //lambda values for adaptive control
	matrix::Vector3f _mfc_gain_p;
	matrix::Vector3f _mfc_gain_i;
	matrix::Vector3f _mfc_gain_d;
	matrix::Vector3f _last_u;                 // to store the last u term
	matrix::Vector3f _current_u;              // current u term
	matrix::Vector3f _measurement;
	matrix::Vector2f _f_hat;
	matrix::Vector3f _sp_double_der;
	rc_channels_s _rc_channel_values;
	float _lambda;  // for mfc
	float _gain_sp;
	float _gain_f_hat;
	int _mfc_n;
	float _mfc_dt;
	float _F_hat_calc;
	float _total_time;
	float _time_steps[22] = {0.0f};
	float _roll_last_u[22] = {0.0f};
	float _pitch_last_u[22] = {0.0f};
	float _roll_sp_values[22] = {0.0f};
	float _pitch_sp_values[22] = {0.0f};
	float _roll_rate_values[22] = {0.0f};
	float _pitch_rate_values[22] = {0.0f};

	// THRUST LOSS PARAETERS
	float _mass = 1.2f; //mass of the drone change it to parameter later
	float _ct = 1.938e-6f; // thrust coefficient
	float _cm = 1.796e-8;  // motor moment coefficient
	float _d = 0.13f; //distance from center to each propeller
	float _root2over2dct = (float(sqrt(2.0))/2.0f) * _d * _ct;
	float _jxy = 0.0133f; // inertia in x and y axis
	float _jz = 0.02587f; // inertia in yaw
	float _lambda_1[2] = {0.5f, 1.0f};  // starting lambda intervals
	float _lambda_2[2] = {0.5f, 1.0f};
	float _lambda_3[2] = {0.5f, 1.0f};
	float _lambda_4[2] = {0.5f, 1.0f};
	bool alternator = true;
	actuator_outputs_s _actuator_output_values;
	vehicle_local_position_s _local_position_values;
};
