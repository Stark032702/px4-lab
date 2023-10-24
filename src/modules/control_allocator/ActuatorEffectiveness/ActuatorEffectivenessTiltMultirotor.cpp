/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessTiltMultirotor.hpp"

using namespace matrix;

ActuatorEffectivenessTiltMultirotor::ActuatorEffectivenessTiltMultirotor(ModuleParams *parent)
	: ModuleParams(parent), _motors(this)
{
}

bool
ActuatorEffectivenessTiltMultirotor::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	_motors.enablePropellerTorque(false);
	const bool motors_added_successfully = _motors.addActuators(configuration);
	_motors_mask = _motors.getMotors();


	return (motors_added_successfully);
}

void ActuatorEffectivenessTiltMultirotor::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	float thrust_x = control_sp(ControlAxis::THRUST_X);
	float thrust_y = control_sp(ControlAxis::THRUST_Y);
	float thrust_z = control_sp(ControlAxis::THRUST_Z);
	float roll_torque = control_sp(ControlAxis::ROLL);
	float pitch_torque = control_sp(ControlAxis::PITCH);
	float yaw_torque = control_sp(ControlAxis::YAW);
	float wrench[6] = {thrust_x, thrust_y, thrust_z, roll_torque, pitch_torque, yaw_torque};
	float sol_array[72] = {-0.5f ,0.0f ,0.0f ,0.1927553f ,0.0f ,-0.3390659f ,
	0.0f ,0.5f ,-0.0f ,-0.0f ,-0.1927553f ,-0.3390659f ,
	-0.0f ,0.0f ,-0.5f ,0.4613517f ,-0.4613517f ,0.1416636f ,
	-0.5f ,0.0f ,0.0f ,0.1927553f ,0.0f ,0.3390659f ,
	-0.0f ,0.5f ,-0.0f ,-0.0f ,-0.1927553f ,0.3390659f ,
	-0.0f ,0.0f ,-0.5f ,-0.4613517f ,0.4613517f ,0.1416636f ,
	-0.5f ,-0.0f ,-0.0f ,-0.1927553f ,-0.0f ,0.3390659f ,
	0.0f ,0.5f ,0.0f ,-0.0f ,0.1927553f ,-0.3390659f ,
	0.0f ,-0.0f ,-0.5f ,-0.4613517f ,-0.4613517f ,-0.1416636f ,
	-0.5f ,-0.0f ,-0.0f ,-0.1927553f ,-0.0f ,-0.3390659f ,
	-0.0f ,0.5f ,0.0f ,0.0f ,0.1927553f ,0.3390659f ,
	0.0f ,0.0f ,-0.5f ,0.4613517f ,0.4613517f ,-0.1416636f};
	matrix::Vector<float, 6> U(wrench);
	matrix::Matrix<float, 12,6> solution(sol_array);
	matrix::Matrix<float, 12, 1> outputs = (solution * U);

	float w_motor1 = (sqrtf(fabsf(outputs(2, 0))))/4.8f;
	float w_motor2 = (sqrtf(fabsf(outputs(5, 0))))/4.8f;
	float w_motor3 = (sqrtf(fabsf(outputs(8, 0))))/4.8f;
	float w_motor4 = (sqrtf(fabsf(outputs(11, 0))))/4.8f;
	/*
	_thrust_rpyt_out[AP_MOTORS_MOT_1] = (w_motor1);
	_thrust_rpyt_out[AP_MOTORS_MOT_2] = (w_motor2);
	_thrust_rpyt_out[AP_MOTORS_MOT_3] = (w_motor3);
	_thrust_rpyt_out[AP_MOTORS_MOT_4] = (w_motor4);
	*/

	actuator_sp(0) = w_motor1;
	actuator_sp(1) = w_motor2;
	actuator_sp(2) = w_motor3;
	actuator_sp(3) = w_motor4;

	actuator_setpoint_s actuator_setpoint_values;
	actuator_setpoint_values.timestamp = hrt_absolute_time();
	actuator_setpoint_values.motor1 = actuator_sp(0);
	actuator_setpoint_values.motor2 = actuator_sp(1);
	_actuator_setpoint_pub.publish(actuator_setpoint_values);

	float pitch1_angle = float(outputs(0,0)/outputs(2,0));
	float pitch2_angle = float(outputs(3,0)/outputs(5,0));
	float pitch3_angle = float(outputs(6,0)/outputs(8,0));
	float pitch4_angle = float(outputs(9,0)/outputs(11,0));
	pitch1_angle =   M_PI_2_F + remainderf(pitch1_angle,M_PI_2_F);
	pitch2_angle =   M_PI_2_F + remainderf(pitch2_angle,M_PI_2_F);
	pitch3_angle =   M_PI_2_F + remainderf(pitch3_angle,M_PI_2_F);
	pitch4_angle =   M_PI_2_F + remainderf(pitch4_angle,M_PI_2_F);
	actuator_sp(4) =  math::degrees(pitch1_angle);
	actuator_sp(5) =  math::degrees(pitch2_angle);
	actuator_sp(6) =  math::degrees(pitch3_angle);
	actuator_sp(7) =   math::degrees(pitch4_angle);
	float roll1_angle = float(outputs(1,0)/outputs(2,0));
	float roll2_angle = float(outputs(4,0)/outputs(5,0));
	float roll3_angle = float(outputs(7,0)/outputs(8,0));
	float roll4_angle = float(outputs(10,0)/outputs(11,0));
	roll1_angle = M_PI_2_F  +  remainderf(roll1_angle,M_PI_2_F);
	roll2_angle = M_PI_2_F  +  remainderf(roll2_angle,M_PI_2_F);
	roll3_angle = M_PI_2_F  +  remainderf(roll3_angle,M_PI_2_F);
	roll4_angle = M_PI_2_F  +  remainderf(roll4_angle,M_PI_2_F);
	actuator_sp(8)  = math::degrees(roll1_angle);
	actuator_sp(9)  = math::degrees(roll2_angle);
	actuator_sp(10)  = math::degrees(roll3_angle);
	actuator_sp(11)  = math::degrees(roll4_angle);
}
