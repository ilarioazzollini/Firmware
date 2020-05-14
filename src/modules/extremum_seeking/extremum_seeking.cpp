/*
 * @author Lorenzo Gentilini (University of Bologna)
 * May, 2020
 */

#include "extremum_seeking.hpp"

// ** ExSeeking Class ** //
// Constructor
ExSeeking::ExSeeking(){
	zPlus 	= 0.0;
	z 		= 0.0;
	fPlus 	= 0.0;
	f 		= 0.0;
	yZ 		= 0.0;
	uX		= 0.0;
	uXPlus	= 0.0;
	uY		= 0.0;
	uYPlus	= 0.0;
	k		= 0.0;
}

// Destructor
ExSeeking::~ExSeeking(){;}

// Main Functions
std::vector<double> ExSeeking::update(double y){
	std::vector<double> x(2);

	// Update Z
	zPlus =  A*z + B*y;
    yZ = - C*z + C*y;

	// Update alpha (Reference pre-filtering)
	fPlus =  FA*f + FB*ALPHA;
    alpha = FC*f + FD*ALPHA;

	// X - Direction
    uXPlus = uX + D*(yZ*alpha*cos(OMEGA*k));
	x.at(0) = uX + E*(yZ*alpha*cos(OMEGA*k)) + alpha*cos(OMEGA*k);

    // Y - Direction
    uYPlus = uY + D*(yZ*alpha*sin(OMEGA*k));
    x.at(1) = uY + E*(yZ*alpha*sin(OMEGA*k)) + alpha*sin(OMEGA*k);

	// Update Memory Variables
    z = zPlus;
    f = fPlus;	
    uX = uXPlus;
    uY = uYPlus;
    k = k + 0.05;

	return x;
}

void ExSeeking::reset(){
	zPlus 	= 0.0;
	z 		= 0.0;
	fPlus 	= 0.0;
	f 		= 0.0;
	yZ 		= 0.0;
	uX		= 0.0;
	uXPlus	= 0.0;
	uY		= 0.0;
	uYPlus	= 0.0;
	k		= 0.0;
}

// -------------------------------------------------------------- //

// ** ESModule Class ** //
// Constructor
ESModule::ESModule() : ModuleParams(nullptr){
	_ref_gen = new ExSeeking;
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_arva_sub = orb_subscribe(ORB_ID(sensor_arva));
}

//Destructor
ESModule::~ESModule(){
	delete _ref_gen;
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_arva_sub);
}

// Main Module Operation --- Takeoff and Search --- //
void ESModule::run(){
	PX4_INFO("Extremum Seeking Module Start");

	struct offboard_control_mode_s 		_offboard_control_mode;
	struct vehicle_local_position_s 	_local_pos;
	struct sensor_arva_s			 	_sens_arva;
	struct position_setpoint_triplet_s	_pos_sp_triplet;
	struct vehicle_command_s 			_vcmd;

	// Set Polling on State & ARVA Topic
	px4_pollfd_struct_t fds[2];	
	fds[0].fd = _arva_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _local_pos_sub;
	fds[1].events = POLLIN;

	// Rate-Limit Subscription to 20Hz (50ms)
	orb_set_interval(_arva_sub, 50);
	orb_set_interval(_local_pos_sub, 50);

	// Arming Vehicle
	_vcmd.param1 	= 1.f;
	_vcmd.param2 	= 0.f;
	_vcmd.param3 	= NAN;
	_vcmd.param4 	= NAN;
	_vcmd.param5 	= NAN;
	_vcmd.param6 	= NAN;
	_vcmd.param7 	= NAN;
	_vcmd.command 	= vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;

	_vcmd.source_system 	= _vehicle_status_sub.get().system_id;
	_vcmd.target_system 	= _vehicle_status_sub.get().system_id;
	_vcmd.source_component 	= _vehicle_status_sub.get().component_id;
	_vcmd.target_component 	= _vehicle_status_sub.get().component_id;
	_vcmd.timestamp 		= hrt_absolute_time();

	_cmd_pub.publish(_vcmd);
	// -- //

	// Fill OffBoard Msg Mode
	_offboard_control_mode.ignore_position 				= false;
	_offboard_control_mode.ignore_alt_hold 				= true;
	_offboard_control_mode.ignore_attitude 				= true;
	_offboard_control_mode.ignore_thrust 				= true;
	_offboard_control_mode.ignore_bodyrate_x			= true;
	_offboard_control_mode.ignore_bodyrate_y			= true;
	_offboard_control_mode.ignore_bodyrate_z			= true;
	_offboard_control_mode.ignore_velocity 				= true;
	_offboard_control_mode.ignore_acceleration_force	= true;
	// -- //

	// Fill OffBoard Cmd
	_vcmd.param1 = 1;
	_vcmd.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
	_vcmd.param3 = NAN;
	_vcmd.param4 = NAN;
	_vcmd.param5 = NAN;
	_vcmd.param6 = NAN;
	_vcmd.param7 = NAN;
	_vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;

	_vcmd.source_system 	= _vehicle_status_sub.get().system_id;
	_vcmd.target_system 	= _vehicle_status_sub.get().system_id;
	_vcmd.source_component 	= _vehicle_status_sub.get().component_id;
	_vcmd.target_component 	= _vehicle_status_sub.get().component_id;
	// -- //

	while(!should_exit()){
		// Publish Offboard Request Constantly 
		// Set Time
		_offboard_control_mode.timestamp = hrt_absolute_time();
		_vcmd.timestamp 				 = hrt_absolute_time();

		// Publish
		_offboard_control_mode_pub.publish(_offboard_control_mode);
		_cmd_pub.publish(_vcmd);
		// -- //

		// Set Maximum Time Of 600ms -> 100ms More
		int count_arva = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 600);
		int count_pose = px4_poll(fds, (sizeof(fds) / sizeof(fds[1])), 600);

	 	if(count_arva < 0) {
			// Error
			PX4_ERR("Poll Error %d, %d", count_arva, errno);
			px4_usleep(600);
			continue;

		} else if(fds[0].revents & POLLIN){
			// ARVA is available
			orb_copy(ORB_ID(sensor_arva), _arva_sub, &_sens_arva);

			y = _sens_arva.y;
		}

		if(count_pose < 0) {
			// Error
			PX4_ERR("Poll Error %d, %d", count_pose, errno);
			px4_usleep(600);
			continue;

		} else if(fds[1].revents & POLLIN){
			// Position is available
			orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

			if(abs((double)_local_pos.z + Z_REF) < EPSILON)
				_state = SEARCH;
		}
		// -- //

		// State Machine For TakeOff and ES Search
		switch (_state){
			case TAKEOFF:
			// Takeoff Triplet
			_pos_sp_triplet.current.x = 0.0;
			_pos_sp_triplet.current.y = 0.0;
			_pos_sp_triplet.current.z = -Z_REF;
			break;

			case SEARCH:
			// Search Triplet
			std::vector<double> ref = _ref_gen->update(y);
			_pos_sp_triplet.current.x 	= ref.at(0);
			_pos_sp_triplet.current.y 	= ref.at(1);
			_pos_sp_triplet.current.z 	= -Z_REF;
			break;
		}

		// Always Publish Set Points //
		// Fill Set Point Msg
		_pos_sp_triplet.previous.valid				= false;
		_pos_sp_triplet.next.valid					= false;
		_pos_sp_triplet.current.valid 				= true;
		_pos_sp_triplet.current.position_valid		= true;
		_pos_sp_triplet.current.acceleration_valid	= false;
		_pos_sp_triplet.current.velocity_valid 		= false;
		_pos_sp_triplet.current.yaw_valid	 		= true;
		_pos_sp_triplet.current.yawspeed_valid 		= false;
		_pos_sp_triplet.current.alt_valid	 		= true;

		_pos_sp_triplet.current.type 				= position_setpoint_s::SETPOINT_TYPE_POSITION;
		_pos_sp_triplet.current.timestamp 			= hrt_absolute_time();
		_pos_sp_triplet.timestamp 					= hrt_absolute_time();

		_pos_sp_triplet.current.yaw					= 0.0;

		// Publish
		_pos_sp_triplet_pub.publish(_pos_sp_triplet);

		// For Debug Purpose --> Publish ARVA Value in Console (At Its Frequency)
		if(abs(y - oldY) > 0.0){
			std::cout << y << std::endl;
			oldY = y;
		}
	}
}

// Module Functions --------------------------------------------------------- //
int ESModule::print_status(){
	PX4_INFO("Running");

	return 0;
}

int ESModule::custom_command(int argc, char *argv[])
{
	if (!is_running()){
		print_usage("Module Not Running");
		return 1;
	}

	return print_usage("Unknown Command");
}

// To Spawn The Application
int ESModule::task_spawn(int argc, char *argv[]){
	_task_id = px4_task_spawn_cmd("es_module", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
				      1800, (px4_main_t)&run_trampoline, (char *const *)argv);

	if (_task_id < 0){
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ESModule *ESModule::instantiate(int argc, char *argv[]){
	ESModule *instance = new ESModule();

	if (instance == nullptr) {
		PX4_ERR("Alloc Failed");
	}

	return instance;
}

int ESModule::print_usage(const char *reason){
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		Extremum Seeking Module.

		This module work with 'start/stop/status' functionality.
		Type 'start' to begin the searching.

		)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("extremum_seeking", "");
	PRINT_MODULE_USAGE_COMMAND("'start'");

	return 0;
}

int extremum_seeking_main(int argc, char *argv[]){
	return ESModule::main(argc, argv);
}
