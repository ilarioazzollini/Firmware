/**
 * @file mc_control_main.cpp
 * Multicopter controller.
 */

#include "mc_control.hpp"

// Constructor
MulticopterControl::MulticopterControl() :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_control(this),
	_cycle_perf(perf_alloc_once(PC_ELAPSED, MODULE_NAME": cycle time")){

	// Initialize Variables
	att.q[0] = 1.f;

	_rates_sp.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	// Fetch initial parameter values
	parameters_update(true);

	// Set failsafe hysteresis
	_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);
}
// ************************************************************************************** //

// Destructor
MulticopterControl::~MulticopterControl(){
	if (_wv_controller != nullptr) {
		delete _wv_controller;
	}

	perf_free(_cycle_perf);
}
// ************************************************************************************** //

// Main Control Function
void MulticopterControl::Run(){
	// Clear All if Exit
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// Run Controller at Frequency Defined Previously - NB: Updating Frequency Should Be > 200Hz
	// Update on GYRO Update 
	actualTime = hrt_absolute_time();
	if(actualTime - previousTime >= 3999){
		previousTime = actualTime;
	//if (_vehicle_angular_velocity_sub.update(&angular_velocity)){
		// Check Immediately Topics and Parameters Update
		_vehicle_angular_velocity_sub.update(&angular_velocity);
		poll_subscriptions();
		parameters_update(false);

		// Set _dt in controllib Block - Time difference since the last loop iteration in seconds
		const hrt_abstime time_stamp_current = hrt_absolute_time();
		setDt((time_stamp_current - _time_stamp_last_loop) / 1e6f);
		_time_stamp_last_loop = time_stamp_current;

		const bool was_in_failsafe = _in_failsafe;
		_in_failsafe = false;

		// Activate the weathervane controller if required.
		// If activated a flighttask can use it to implement a yaw-rate control strategy that turns the nose of the vehicle into the wind
		if (_wv_controller != nullptr) {
			// In manual mode we just want to use weathervane if position is controlled as well
			// In mission, enabling wv is done in flight task
			if (_control_mode.flag_control_manual_enabled) {
				if (_control_mode.flag_control_position_enabled && _wv_controller->weathervane_enabled()) {
					_wv_controller->activate();

				} else {
					_wv_controller->deactivate();
				}
			}
			_wv_controller->update(matrix::Quatf(_att_sp.q_d), _states.yaw);
		}

		// An update is necessary here because otherwise the takeoff state doesn't get skiped with non-altitude-controlled modes
		_takeoff.updateTakeoffState(_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f,
					    		   !_control_mode.flag_control_climb_rate_enabled, time_stamp_current);

		// Takeoff delay for motors to reach idle speed
		if (_takeoff.getTakeoffState() >= TakeoffState::ready_for_takeoff) {
			// When vehicle is ready switch to the required flighttask
			start_flight_task();

		} else {
			// Stop flighttask while disarmed
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}

		// Check if any task is active
		if (_flight_tasks.isAnyTaskActive()) {
			// Setpoints and constraints for the position controller from flighttask or failsafe
			vehicle_local_position_setpoint_s setpoint = FlightTask::empty_setpoint;
			vehicle_constraints_s constraints = FlightTask::empty_constraints;

			_flight_tasks.setYawHandler(_wv_controller);

			// Update task
			if (!_flight_tasks.update()) {
				// FAILSAFE
				// Task was not able to update correctly. Do Failsafe.
				failsafe(setpoint, _states, false, !was_in_failsafe);

			} else {
				setpoint = _flight_tasks.getPositionSetpoint();
				constraints = _flight_tasks.getConstraints();

				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_current);

				// Check if position, velocity or thrust pairs are valid -> trigger failsaife if no pair is valid
				if (!(PX4_ISFINITE(setpoint.x) && PX4_ISFINITE(setpoint.y)) &&
				    !(PX4_ISFINITE(setpoint.vx) && PX4_ISFINITE(setpoint.vy)) &&
				    !(PX4_ISFINITE(setpoint.thrust[0]) && PX4_ISFINITE(setpoint.thrust[1]))) {
					failsafe(setpoint, _states, true, !was_in_failsafe);
				}

				// Check if altitude, climbrate or thrust in D-direction are valid -> trigger failsafe if none of these setpoints are valid
				if (!PX4_ISFINITE(setpoint.z) && !PX4_ISFINITE(setpoint.vz) && !PX4_ISFINITE(setpoint.thrust[2])) {
					failsafe(setpoint, _states, true, !was_in_failsafe);
				}
			}

			// Publish trajectory setpoint
			_traj_sp_pub.publish(setpoint);

			landing_gear_s gear = _flight_tasks.getGear();

			// Check if all local states are valid and map accordingly
			set_vehicle_states(setpoint.vz);

			// Handle smooth takeoff
			_takeoff.updateTakeoffState(_control_mode.flag_armed, _vehicle_land_detected.landed, constraints.want_takeoff,
						    			constraints.speed_up, !_control_mode.flag_control_climb_rate_enabled, time_stamp_current);
			constraints.speed_up = _takeoff.updateRamp(_dt, constraints.speed_up);

			if (_takeoff.getTakeoffState() < TakeoffState::rampup && !PX4_ISFINITE(setpoint.thrust[2])) {
				// We are not flying yet and need to avoid any corrections
				reset_setpoint_to_nan(setpoint);
				setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = 0.0f;

				// Set yaw-sp to current yaw
				setpoint.yaw = _states.yaw;
				setpoint.yawspeed = 0.f;

				// Prevent any integrator windup
				// TODO: Eliminate It
				//_control.resetIntegralXY();
				//_control.resetIntegralZ();

				// Reactivate the task which will reset the setpoint to current state
				_flight_tasks.reActivate();
			}

			if (_takeoff.getTakeoffState() < TakeoffState::flight && !PX4_ISFINITE(setpoint.thrust[2])) {
				constraints.tilt = math::radians(_param_mpc_tiltmax_lnd.get());
			}

			// Limit altitude only if local position is valid
			if (PX4_ISFINITE(_states.position(2))) {
				limit_altitude(setpoint);
			}

			// Information For Attitude/Rate Controller
			const Vector3f rates{angular_velocity.xyz};
			const Matrix<float, 4, 1> quaternion{att.q};
			_actuators.timestamp_sample = angular_velocity.timestamp_sample;

			// Position Control // *************************** //
			// Update states, setpoints and constraints.
			//_control.updateConstraints(constraints);
			//_control.updateState(_states);

			// Update position controller setpoints
			/*if (!_control.updateSetpoint(setpoint)) {
				warn_rate_limited("Position-Control Setpoint-Update failed");
				failsafe(setpoint, _states, true, !was_in_failsafe);
				_control.updateSetpoint(setpoint);
				constraints = FlightTask::empty_constraints;
			}*/

			// Generate desired thrust and yaw.
			//_control.generateThrustYawSetpoint(_dt);
			// Position Control // *************************** //

			// Extract Info From Position Control
			// Fill local position, velocity and thrust setpoint.
			/*vehicle_local_position_setpoint_s local_pos_sp{};
			local_pos_sp.timestamp = hrt_absolute_time();
			local_pos_sp.x = setpoint.x;
			local_pos_sp.y = setpoint.y;
			local_pos_sp.z = setpoint.z;
			local_pos_sp.yaw = _control.getYawSetpoint();
			local_pos_sp.yawspeed = _control.getYawspeedSetpoint();
			local_pos_sp.vx = PX4_ISFINITE(_control.getVelSp()(0)) ? _control.getVelSp()(0) : setpoint.vx;
			local_pos_sp.vy = PX4_ISFINITE(_control.getVelSp()(1)) ? _control.getVelSp()(1) : setpoint.vy;
			local_pos_sp.vz = PX4_ISFINITE(_control.getVelSp()(2)) ? _control.getVelSp()(2) : setpoint.vz;
			_control.getThrustSetpoint().copyTo(local_pos_sp.thrust);*/

			// Publish local position setpoint
			// This message will be used by other modules (such as Landdetector) to determine vehicle intention.
			//_local_pos_sp_pub.publish(local_pos_sp);

			// Inform FlightTask about the input and output of the velocity controller
			// This is used to properly initialize the velocity setpoint when onpening the position loop (position unlock)
			//_flight_tasks.updateVelocityControllerIO(_control.getVelSp(), Vector3f(local_pos_sp.thrust));

			// Part of landing logic: if ground-contact/maybe landed was detected, turn off
			// controller. This message does not have to be logged as part of the vehicle_local_position_setpoint topic.
			// Note: only adust thrust output if there was not thrust-setpoint demand in D-direction.
			//if (_takeoff.getTakeoffState() > TakeoffState::rampup && !PX4_ISFINITE(setpoint.thrust[2])) {
			//	limit_thrust_during_landing(local_pos_sp);
			//}

			// Extract Attitude SetPoints
			//_att_sp = ControlMath::thrustToAttitude(matrix::Vector3f(local_pos_sp.thrust), local_pos_sp.yaw);
			//_att_sp.yaw_sp_move_rate = _control.getYawspeedSetpoint();
			//_att_sp.fw_control_yaw = false;
			//_att_sp.apply_flaps = false;

			// Attitude/Rate Control // *************************** //
			// Run Attitude Control
			//control_attitude();

			// Run Rate Control
			//control_attitude_rates(_dt, rates);

			// IMB Control
			// Fill State and SetPoint MSGS
			matrix::Matrix<double, 12, 1> actualState;
			matrix::Matrix<double, 3, 1> poseSetPoint;

			//std::cout << quaternion(0,0) << " " << quaternion(1,0) << " " << quaternion(2,0) << " " << quaternion(3,0) << std::endl;

			double phi = atan2(2*(quaternion(0,0)*quaternion(1,0) + quaternion(2,0)*quaternion(3,0)),
									1 - 2*(pow(quaternion(1,0), 2) + pow(quaternion(2,0), 2)));

			double theta = asin(2*(quaternion(0,0)*quaternion(2,0) - quaternion(3,0)*quaternion(1,0)));
			
			double psi = atan2(2*(quaternion(0,0)*quaternion(3,0) + quaternion(1,0)*quaternion(2,0)),
									1 - 2*(pow(quaternion(2,0), 2) + pow(quaternion(3,0), 2)));

			// States are Memorized As
    		// 0:x / 1:y / 2:z / 3:xDot / 4:yDot / 5:zDot
    		// 6:phi / 7:theta / 8:psi / 9:phiDot / 10:thetaDot / 11:psiDot
			
			actualState(0,0) = _local_pos.x; actualState(1,0) = _local_pos.y; actualState(2,0) = _local_pos.z;
			actualState(3,0) = _local_pos.vx; actualState(4,0) = _local_pos.vy; actualState(5,0) = _local_pos.vz;
			actualState(6,0) = phi; actualState(7,0) = theta; actualState(8,0) = psi;
			actualState(9,0) = rates(0); actualState(10,0) = rates(1); actualState(11,0) = rates(2);

			poseSetPoint(0,0) = setpoint.x; poseSetPoint(1,0) = setpoint.y; poseSetPoint(2,0) = setpoint.z;

			// Publish Errors
			_errors.ex = _local_pos.x - setpoint.x;
			_errors.ey = _local_pos.y - setpoint.y;
			_errors.ez = _local_pos.z - setpoint.z;
			_errors.timestamp = hrt_absolute_time();
			_errors_pub.publish(_errors);

			// Update Control
			matrix::Matrix<double, 4, 1> controlAction = _IMBControl.update(actualState, poseSetPoint);

			//controlAction(0,0) /= 4;
			//controlAction(1,0) /= 4;
			//controlAction(2,0) /= 4;
			controlAction(3,0) /= 27.15;

			if(controlAction(0,0) > 0){
				_att_control(0) = controlAction(0,0)>1 ? 1.0 : controlAction(0,0);
			} else{
				_att_control(0) = controlAction(0,0)<-1 ? -1.0 : controlAction(0,0);
			}

			if(controlAction(1,0) > 0){
				_att_control(1) = controlAction(1,0)>1 ? 1.0 : controlAction(1,0);
			} else{
				_att_control(1) = controlAction(1,0)<-1 ? -1.0 : controlAction(1,0);
			}

			if(controlAction(2,0) > 0){
				_att_control(2) = controlAction(2,0)>1 ? 1.0 : controlAction(2,0);
			} else{
				_att_control(2) = controlAction(2,0)<-1 ? -1.0 : controlAction(2,0);
			}

			if(controlAction(3,0) > 0){
				_thrust_sp = controlAction(3,0)>1 ? 1.0 : controlAction(3,0);
			} else{
				_thrust_sp = 0.0;
			}

			//std::cout << _att_control(0) << " " << _att_control(1) << " " << _att_control(2) << std::endl;
			//std::cout << _thrust_sp << std::endl;
			
			
			// Publish Actuator Controls
			publish_actuator_controls();
			//publish_rate_controller_status();

			// Attitude/Rate Control // *************************** //

			// If Termination is TRUE -> Stop Control and Set All Zero
			if (_control_mode.flag_control_termination_enabled) {
				if (!_vehicle_status.is_vtol) {
					_rates_sp.zero();
					_rate_control.resetIntegral();
					_thrust_sp = 0.0f;
					_att_control.zero();
					publish_actuator_controls();
				}
			}

			// Calculate loop update rate while disarmed or at least a few times (updating the filter is expensive)
			// Actually this is not fundamental !!!
			if (!_control_mode.flag_armed || (time_stamp_current - _task_start) < 3300000) {
				_dt_accumulator += _dt;
				++_loop_counter;
				if (_dt_accumulator > 1.f) {
					const float loop_update_rate = (float)_loop_counter / _dt_accumulator;
					_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
					_dt_accumulator = 0;
					_loop_counter = 0;
					//_rate_control.setDTermCutoff(_loop_update_rate_hz, _param_mc_dterm_cutoff.get(), true);
				}

				_IMBControl.reset();
			}

			// If there's any change in landing gear setpoint publish it
			if (gear.landing_gear != _old_landing_gear_position
			    && gear.landing_gear != landing_gear_s::GEAR_KEEP) {

				_landing_gear.landing_gear = gear.landing_gear;
				_landing_gear.timestamp = hrt_absolute_time();

				_landing_gear_pub.publish(_landing_gear);
			}

			_old_landing_gear_position = gear.landing_gear;

		} else {
			// No flighttask is active: set attitude setpoint to idle
			_att_sp.roll_body = _att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = _states.yaw;
			_att_sp.yaw_sp_move_rate = 0.0f;
			_att_sp.fw_control_yaw = false;
			_att_sp.apply_flaps = false;
			matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
			q_sp.copyTo(_att_sp.q_d);
			_att_sp.q_d_valid = true;
			_att_sp.thrust_body[2] = 0.0f;

			// Reset the numerical derivatives to not generate d term spikes when coming from non-position controlled operation
			_vel_x_deriv.reset();
			_vel_y_deriv.reset();
			_vel_z_deriv.reset();
		}
	}

	perf_end(_cycle_perf);
}
// ************************************************************************************** //

//////////////////////////////////////////////////////////////////////////// Helper Functions

// Publish Warning at Lower Rate
void MulticopterControl::warn_rate_limited(const char *string){
	hrt_abstime now = hrt_absolute_time();

	if (now - _last_warn > 200_ms) {
		PX4_WARN("%s", string);
		_last_warn = now;
	}
}
// ************************************************************************************** //

// Check for Parameters Update
int MulticopterControl::parameters_update(bool force){
	// Check for Parameter Updates
	if (_parameter_update_sub.updated()) {
		// Clear Update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// Update Parameters from Storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		// Check Parameters Validity
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed");
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed");
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(), _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max");
		}

		_flight_tasks.handleParameterUpdate();

		// Initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		// Set trigger time for takeoff delay
		_takeoff.setSpoolupTime(_param_mpc_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_thr_hover.get(), _param_mpc_z_vel_p.get());

		if (_wv_controller != nullptr) {
			_wv_controller->update_parameters();
		}

		// Attitude/Rate Parameters
		_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()));

		// Rate Control Parameters
		// The controller gain K is used to convert the parallel (P + I/s + sD) form to the ideal (K * [1 + 1/sTi + sTd]) form
		Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());
		
		_rate_control.setGains(rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
							   rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
							   rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));
		_rate_control.setIntegratorLimit(Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));
		_rate_control.setDTermCutoff(_loop_update_rate_hz, _param_mc_dterm_cutoff.get(), false);
		_rate_control.setFeedForwardGain(Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));

		// Angular Rate Limits
		using math::radians;
		_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()), radians(_param_mc_yawrate_max.get())));

		_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
	}

	return OK;
}
// ************************************************************************************** //

// Check Topics Update
void MulticopterControl::poll_subscriptions(){
	_local_pos_sub.update(&_local_pos);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_control_mode_sub.update(&_control_mode);
	_home_pos_sub.update(&_home_pos);

	/* check for updates in other topics */
	_battery_status_sub.update(&_battery_status);
	vehicle_status_poll();
	vehicle_motor_limits_poll();
	vehicle_attitude_poll();
}
// ************************************************************************************** //

// Limit Altitude
void MulticopterControl::limit_altitude(vehicle_local_position_setpoint_s &setpoint){
	if (_vehicle_land_detected.alt_max < 0.0f || !_home_pos.valid_alt || !_local_pos.v_z_valid) {
		// There is no altitude limitation present or the required information not available
		return;
	}

	const float min_z = _home_pos.z + (-_vehicle_land_detected.alt_max);

	if (_states.position(2) < min_z) {
		// Above maximum altitude, only allow downwards flight == positive vz-setpoints (NED)
		setpoint.z = min_z;
		setpoint.vz = math::max(setpoint.vz, 0.f);
	}
}
// ************************************************************************************** //

// Check for validity of positon/velocity states.
void MulticopterControl::set_vehicle_states(const float &vel_sp_z){
	if (_local_pos.timestamp == 0) {
		return;
	}

	// Only set position states if valid and finite
	if (PX4_ISFINITE(_local_pos.x) && PX4_ISFINITE(_local_pos.y) && _local_pos.xy_valid) {
		_states.position(0) = _local_pos.x;
		_states.position(1) = _local_pos.y;

	} else {
		_states.position(0) = _states.position(1) = NAN;
	}

	if (PX4_ISFINITE(_local_pos.z) && _local_pos.z_valid) {
		_states.position(2) = _local_pos.z;

	} else {
		_states.position(2) = NAN;
	}

	if (PX4_ISFINITE(_local_pos.vx) && PX4_ISFINITE(_local_pos.vy) && _local_pos.v_xy_valid) {
		_states.velocity(0) = _local_pos.vx;
		_states.velocity(1) = _local_pos.vy;
		_states.acceleration(0) = _vel_x_deriv.update(-_states.velocity(0));
		_states.acceleration(1) = _vel_y_deriv.update(-_states.velocity(1));

	} else {
		_states.velocity(0) = _states.velocity(1) = NAN;
		_states.acceleration(0) = _states.acceleration(1) = NAN;

		// Since no valid velocity, update derivate with 0
		_vel_x_deriv.update(0.0f);
		_vel_y_deriv.update(0.0f);
	}

	if (_param_mpc_alt_mode.get() && _local_pos.dist_bottom_valid && PX4_ISFINITE(_local_pos.dist_bottom_rate)) {
		// Terrain following
		_states.velocity(2) = -_local_pos.dist_bottom_rate;
		_states.acceleration(2) = _vel_z_deriv.update(-_states.velocity(2));

	} else if (PX4_ISFINITE(_local_pos.vz)) {

		_states.velocity(2) = _local_pos.vz;

		if (PX4_ISFINITE(vel_sp_z) && fabsf(vel_sp_z) > FLT_EPSILON && PX4_ISFINITE(_local_pos.z_deriv)) {
			// A change in velocity is demanded. Set velocity to the derivative of position
			// because it has less bias but blend it in across the landing speed range
			float weighting = fminf(fabsf(vel_sp_z) / _param_mpc_land_speed.get(), 1.0f);
			_states.velocity(2) = _local_pos.z_deriv * weighting + _local_pos.vz * (1.0f - weighting);
		}

		_states.acceleration(2) = _vel_z_deriv.update(-_states.velocity(2));

	} else {
		_states.velocity(2) = _states.acceleration(2) = NAN;
		// since no valid velocity, update derivate with 0
		_vel_z_deriv.update(0.0f);

	}
}
// ************************************************************************************** //

// Start flightasks based on navigation state.
void MulticopterControl::start_flight_task(){
	bool task_failure = false;
	bool should_disable_task = true;
	int prev_failure_count = _task_failure_count;

	// Do not run any flight task for VTOLs in fixed-wing mode
	if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
		return;
	}

	if (_vehicle_status.in_transition_mode) {
		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Transition);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Transition activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// We want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

		return;
	}

	// Offboard
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	    && (_control_mode.flag_control_altitude_enabled ||
		_control_mode.flag_control_position_enabled ||
		_control_mode.flag_control_climb_rate_enabled ||
		_control_mode.flag_control_velocity_enabled ||
		_control_mode.flag_control_acceleration_enabled)) {

		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Offboard);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Offboard activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// We want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}

	// Auto-follow me
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) {
		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::AutoFollowMe);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Follow-Me activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// We want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_control_mode.flag_control_auto_enabled) {
		// Auto related maneuvers
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_auto_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::AutoLineSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::AutoLine);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Auto activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// We want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {

		// Emergency descend
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error =  _flight_tasks.switchTask(FlightTaskIndex::Descend);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Descend activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// We want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	}

	// Manual Position Control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmooth);
			break;

		case 2:
			error =  _flight_tasks.switchTask(FlightTaskIndex::Sport);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPosition);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Position-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_POSCTL);
			task_failure = false;
		}
	}

	// Manual Altitude Control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmooth);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitude);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Altitude-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_ALTCTL);
			task_failure = false;
		}
	}

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT) {
		should_disable_task = false;
	}

	// Check Task Failure
	if (task_failure) {

		// For some reason no flighttask was able to start.
		// Go into failsafe flighttask
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Failsafe);

		if (error != FlightTaskError::NoError) {
			// No task was activated.
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}

	} else if (should_disable_task) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
	}
}
// ************************************************************************************** //

// Adjust the setpoint during landing.
void MulticopterControl::limit_thrust_during_landing(vehicle_local_position_setpoint_s &setpoint){
	if (_vehicle_land_detected.ground_contact
	    || _vehicle_land_detected.maybe_landed) {
		// We set thrust to zero, this will help to decide if we are actually landed or not
		setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = 0.0f;
		// Set yaw-sp to current yaw to avoid any corrections
		setpoint.yaw = _states.yaw;
		setpoint.yawspeed = 0.f;

		// Prevent any integrator windup
		// TODO: Remove It!!!
		_control.resetIntegralXY();
		_control.resetIntegralZ();
	}
}
// ************************************************************************************** //

// Failsafe Handling
void MulticopterControl::failsafe(vehicle_local_position_setpoint_s &setpoint, const PositionControlStates &states,
				     const bool force, const bool warn){
	_failsafe_land_hysteresis.set_state_and_update(true, hrt_absolute_time());

	if (!_failsafe_land_hysteresis.get_state() && !force) {
		// Just keep current setpoint and don't do anything.

	} else {
		reset_setpoint_to_nan(setpoint);

		if (PX4_ISFINITE(_states.velocity(0)) && PX4_ISFINITE(_states.velocity(1))) {
			// Don't move along xy
			setpoint.vx = setpoint.vy = 0.0f;

			if (warn) {
				PX4_WARN("Failsafe: stop and wait");
			}

		} else {
			// Descend with land speed since we can't stop
			setpoint.thrust[0] = setpoint.thrust[1] = 0.f;
			setpoint.vz = _param_mpc_land_speed.get();

			if (warn) {
				PX4_WARN("Failsafe: blind land");
			}
		}

		if (PX4_ISFINITE(_states.velocity(2))) {
			// Don't move along z if we can stop in all dimensions
			if (!PX4_ISFINITE(setpoint.vz)) {
				setpoint.vz = 0.f;
			}

		} else {
			// Emergency descend with a bit below hover thrust
			setpoint.vz = NAN;
			setpoint.thrust[2] = _param_mpc_thr_hover.get() * .8f;

			if (warn) {
				PX4_WARN("Failsafe: blind descend");
			}
		}

		_in_failsafe = true;
	}
}
// ************************************************************************************** //

// Reset setpoints to NAN
void MulticopterControl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint){
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acc_x = setpoint.acc_y = setpoint.acc_z = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}
// ************************************************************************************** //

// Check if task should be switched because of failsafe
void MulticopterControl::check_failure(bool task_failure, uint8_t nav_state){
	if (!task_failure) {
		// We want to be in this mode, reset the failure count
		_task_failure_count = 0;

	} else if (_task_failure_count > NUM_FAILURE_TRIES) {
		// Tell commander to switch mode
		PX4_WARN("Previous flight task failed, switching to mode %d", nav_state);
		send_vehicle_cmd_do(nav_state);
		_task_failure_count = 0; // avoid immediate resending of a vehicle command in the next iteration
	}
}
// ************************************************************************************** //

// Send vehicle command to inform commander about failsafe
void MulticopterControl::send_vehicle_cmd_do(uint8_t nav_state){
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1;
	command.param3 = (float)0;
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;

	// Set the main mode
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_STAB:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
		command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	default: //vehicle_status_s::NAVIGATION_STATE_POSCTL
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;
	}

	// Publish the vehicle command
	_pub_vehicle_command.publish(command);
}
// ************************************************************************************** //

// Attitude rates controller.
void MulticopterControl::control_attitude_rates(float dt, const Vector3f &rates){
	// reset integral if disarmed
	if (!_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_rate_control.resetIntegral();
	}

	const bool landed = _vehicle_land_detected.maybe_landed || _vehicle_land_detected.landed;
	_rate_control.setSaturationStatus(_saturation_status);
	_att_control = _rate_control.update(rates, _rates_sp, dt, landed);
}
// ************************************************************************************** //

// Publish Controls to Actuators
void MulticopterControl::publish_actuator_controls(){
	_actuators.control[0] = _att_control(0);
	_actuators.control[1] = _att_control(1);
	_actuators.control[2] = _att_control(2);
	_actuators.control[3] = _thrust_sp;
	_actuators.control[7] = (float)_landing_gear.landing_gear;
	// note: _actuators.timestamp_sample is set in MulticopterAttitudeControl::Run()
	_actuators.timestamp = hrt_absolute_time();

	/*matrix::Matrix<double, 4, 4> mixer;
	matrix::Matrix<double, 4, 1> attControl;
	matrix::Matrix<double, 4, 1> attOutput;

	mixer(0,0) = 1; mixer(0,1) = -1; mixer(0,2) = 1; mixer(0,3) = 1;
	mixer(1,0) = 1; mixer(1,1) = 1; mixer(1,2) = -1; mixer(1,3) = 1;
	mixer(2,0) = 1; mixer(2,1) = -1; mixer(2,2) = -1; mixer(2,3) = -1;
	mixer(3,0) = 1; mixer(3,1) = 1; mixer(3,2) = 1; mixer(3,3) = -1;

	attControl(0,0) = _thrust_sp;
	attControl(1,0) = _att_control(1);
	attControl(2,0) = _att_control(0);
	attControl(3,0) = _att_control(2);


	attOutput = mixer*attControl*1000;

	actuator_outputs_s actuator_outputs;

	actuator_outputs.noutputs = 4;

	actuator_outputs.output[1] = attOutput(0,0);
	actuator_outputs.output[0] = attOutput(1,0);
	actuator_outputs.output[3] = attOutput(2,0);
	actuator_outputs.output[2] = attOutput(3,0);

	actuator_outputs.timestamp = hrt_absolute_time();
	_outputs_pub.publish(actuator_outputs);*/

	/*
	// Scale effort by battery status
	if (_param_mc_bat_scale_en.get() && _battery_status.scale > 0.0f) {
		for (int i = 0; i < 4; i++) {
			_actuators.control[i] *= _battery_status.scale;
		}
	}*/

	if (!_actuators_0_circuit_breaker_enabled) {
		orb_publish_auto(_actuators_id, &_actuators_0_pub, &_actuators, nullptr, ORB_PRIO_DEFAULT);
	}
}
// ************************************************************************************** //

// Publish Controller Status
void MulticopterControl::publish_rate_controller_status(){
	rate_ctrl_status_s rate_ctrl_status = {};
	rate_ctrl_status.timestamp = hrt_absolute_time();
	_rate_control.getRateControlStatus(rate_ctrl_status);
	_controller_status_pub.publish(rate_ctrl_status);
}
// ************************************************************************************** //

// Polling Functions
void MulticopterControl::vehicle_status_poll(){
	// Check if there is new status information
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		// Set correct uORB ID, depending on if vehicle is VTOL or not
		if (_actuators_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}

		// If vehicle is a VTOL we want to enable weathervane capabilities
		if (_wv_controller == nullptr && _vehicle_status.is_vtol) {
			_wv_controller = new WeatherVane();
		}
	}
}

void MulticopterControl::vehicle_motor_limits_poll(){
	// Check if there is a new message
	multirotor_motor_limits_s motor_limits{};

	if (_motor_limits_sub.update(&motor_limits)) {
		_saturation_status.value = motor_limits.saturation_status;
	}
}

bool MulticopterControl::vehicle_attitude_poll(){
	// Check if there is a new message
	const uint8_t prev_quat_reset_counter = att.quat_reset_counter;

	if (_att_sub.update(&att)) {
		// Check for a heading reset
		if (prev_quat_reset_counter != att.quat_reset_counter) {
			// we only extract the heading change from the delta quaternion
			_man_yaw_sp += Eulerf(Quatf(att.delta_q_reset)).psi();
		}

		if (PX4_ISFINITE(att.q[0])) {
			_states.yaw = Eulerf(Quatf(att.q)).psi();
		}

		return true;
	}

	return false;
}
// ************************************************************************************** //


// Attitude controller.
void MulticopterControl::control_attitude(){
	// Reinitialize the setpoint while not armed to make sure no value from the last mode or flight is still kept
	if (!_control_mode.flag_armed) {
		Quatf().copyTo(_att_sp.q_d);
		Vector3f().copyTo(_att_sp.thrust_body);
	}

	// Physical thrust axis is the negative of body z axis
	_thrust_sp = -_att_sp.thrust_body[2];
	_rates_sp = _attitude_control.update(Quatf(att.q), Quatf(_att_sp.q_d), _att_sp.yaw_sp_move_rate);
}
// ************************************************************************************** //

/////////////////////////////////////////////////////////////////////////////////// ModuleBase Functions
bool MulticopterControl::init(){
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("Callback Registration Failed!");
		return false;
	}

	// To Set a Given Updating Frequency
	//_vehicle_angular_velocity_sub.set_interval_us(4_ms);

	// Variable Initialization
	_time_stamp_last_loop = hrt_absolute_time();

	return true;
}

int MulticopterControl::print_status(){
	if (_flight_tasks.isAnyTaskActive()) {
		PX4_INFO("Running, active flight task: %i", _flight_tasks.getActiveTask());

	} else {
		PX4_INFO("Running, no flight task active");
	}

	perf_print_counter(_cycle_perf);

	print_message(_actuators);

	return 0;
}

int MulticopterControl::task_spawn(int argc, char *argv[]){
	MulticopterControl *instance = new MulticopterControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterControl::custom_command(int argc, char *argv[]){
	return print_usage("unknown command");
}

int MulticopterControl::print_usage(const char *reason){
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Multi Copter Controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int mc_control_main(int argc, char *argv[]){
	return MulticopterControl::main(argc, argv);
}
// ************************************************************************************** //