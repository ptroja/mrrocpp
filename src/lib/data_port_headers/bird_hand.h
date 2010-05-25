/*
 **  BIRD_HAND.H
 */

#if !defined(__BIRD_HAND_DATA_PORT_H)
#define __BIRD_HAND_DATA_PORT_H

#define BIRD_HAND_STEP_TIME_IN_NS 2000000

namespace mrrocpp {
namespace lib {

#define BIRD_HAND_COMMAND_DATA_PORT "bird_hand_command_data_port"
#define BIRD_HAND_STATUS_DATA_REQUEST_PORT "bird_hand_status_data_request_port"

#define BIRD_HAND_CONFIGURATION_DATA_PORT "bird_hand_configuration_data_port"
#define BIRD_HAND_CONFIGURATION_DATA_REQUEST_PORT "bird_hand_configuration_data_request_port"

#define BIRD_HAND_NUM_OF_SERVOS	8

enum BIRD_HAND_MOTION_VARIANT {
	BIRD_HAND_SIGLE_STEP_POSTION_INCREMENT = 0,
	BIRD_HAND_MACROSTEP_POSITION_INCREMENT = 1,
	BIRD_HAND_MACROSTEP_ABSOLUTE_POSITION = 2
};

struct bird_hand_command {
	int motion_steps;
	int ecp_query_step;
	BIRD_HAND_MOTION_VARIANT profile_type[BIRD_HAND_NUM_OF_SERVOS];
	double reciprocal_of_damping[BIRD_HAND_NUM_OF_SERVOS];
	double desired_torque[BIRD_HAND_NUM_OF_SERVOS];
	double desired_position[BIRD_HAND_NUM_OF_SERVOS];
};

struct bird_hand_status {
	double meassured_position[BIRD_HAND_NUM_OF_SERVOS];
	double meassured_torque[BIRD_HAND_NUM_OF_SERVOS];
	double meassured_current[BIRD_HAND_NUM_OF_SERVOS];
	bool
			upper_limit_of_absolute_value_of_desired_position_increment[BIRD_HAND_NUM_OF_SERVOS];
	bool
			upper_limit_of_absolute_value_of_computed_position_increment[BIRD_HAND_NUM_OF_SERVOS];
	bool upper_limit_of_absolute_position[BIRD_HAND_NUM_OF_SERVOS];
	bool lower_limit_of_absolute_position[BIRD_HAND_NUM_OF_SERVOS];
	bool
			upper_limit_of_absolute_value_of_desired_torque[BIRD_HAND_NUM_OF_SERVOS];
	bool
			lower_limit_of_absolute_value_of_desired_torque[BIRD_HAND_NUM_OF_SERVOS];
	bool
			upper_limit_of_absolute_value_of_meassured_torque[BIRD_HAND_NUM_OF_SERVOS];
	bool upper_limit_of_meassured_current[BIRD_HAND_NUM_OF_SERVOS];
};

struct bird_hand_configuration {
	int p_factor[BIRD_HAND_NUM_OF_SERVOS];
	int i_factor[BIRD_HAND_NUM_OF_SERVOS];
	int d_factor[BIRD_HAND_NUM_OF_SERVOS];
	int value_of_upper_limit_of_absolute_position[BIRD_HAND_NUM_OF_SERVOS];
	int value_of_lower_limit_of_absolute_position[BIRD_HAND_NUM_OF_SERVOS];
	int value_of_upper_limit_of_meassured_current[BIRD_HAND_NUM_OF_SERVOS];
	int
			value_of_upper_limit_of_absolute_value_of_torque[BIRD_HAND_NUM_OF_SERVOS];
	int
			value_of_lower_limit_of_absolute_value_of_torque[BIRD_HAND_NUM_OF_SERVOS];
	int
			value_of_lower_limit_of_absolute_value_of_meassured_torque[BIRD_HAND_NUM_OF_SERVOS];
	int value_of_upper_limit_of_position_increment[BIRD_HAND_NUM_OF_SERVOS];
};

}
}

#endif
