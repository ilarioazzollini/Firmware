/*
 * @author Lorenzo Gentilini (University of Bologna)
 * May, 2020
 */

#pragma once

#include <px4_module.h>
#include <px4_module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/PublicationQueued.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/sensor_arva.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/uORB.h>

#include "px4_custom_mode.h"

#include <math.h>
#include <iostream>
#include <vector>

#define Z_REF 5
#define CONST 1

#define A 0.9988
#define B 0.0012
#define C 0.9994
#define D -0.0025
#define E -0.0013

#define FA 0.9975
#define FB 0.0625
#define FC 0.0399
#define FD 0.0012

#define ALPHA 2
#define OMEGA 0.5
#define EPSILON 0.1

typedef enum{TAKEOFF, SEARCH} uav_state;

extern "C" __EXPORT int extremum_seeking_main(int argc, char *argv[]);

class ExSeeking{
	private:
	double zPlus, z, yZ;
	double fPlus, f, alpha;
	double uX, uXPlus, uY, uYPlus;
	double k;

	public:
	// Constructor
	ExSeeking();

	// Destructor
	~ExSeeking();

	// Main Functions
	std::vector<double> update(double y);

	void reset();
};

class ESModule : public ModuleBase<ESModule>, public ModuleParams{
	public:
		// Constructor
		ESModule();

		// Destructor
		~ESModule();

		// Module Base Fuctions
		static int task_spawn(int argc, char *argv[]);

		static ESModule *instantiate(int argc, char *argv[]);

		static int custom_command(int argc, char *argv[]);

		static int print_usage(const char *reason = nullptr);

		void run() override;

		int print_status() override;

	private:
		// Attributes
		ExSeeking* _ref_gen;
		uav_state _state;
		int _local_pos_sub, _arva_sub;
		double y = 0.0, oldY = 0.0;

		// Subscriptions
		uORB::Subscription						 _parameter_update_sub{ORB_ID(parameter_update)};
		uORB::Subscription						 _control_mode_sub{ORB_ID(vehicle_control_mode)};
		uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};

		// Publications
		uORB::Publication<position_setpoint_triplet_s> 	_pos_sp_triplet_pub{ORB_ID(position_setpoint_triplet)};
		uORB::Publication<offboard_control_mode_s>		_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
		uORB::PublicationQueued<vehicle_command_s> 		_cmd_pub{ORB_ID(vehicle_command)};
};

