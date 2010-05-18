/*
 **  BIRD_HAND.H
 */

#if !defined(__BIRD_HAND_DATA_PORT_H)
#define __BIRD_HAND_DATA_PORT_H

namespace mrrocpp {
namespace lib {

#define BIRD_HAND_COMMAND_DATA_PORT "bird_hand_configuration_data_port"
#define BIRD_HAND_STATUS_DATA_REQUEST_PORT "bird_hand_status_data_request_port"

#define BIRD_HAND_CONFIGURATION_DATA_PORT "bird_hand_configuration_data_port"
#define BIRD_HAND_CONFIGURATION_DATA_REQUEST_PORT "bird_hand_configuration_data_request_port"

#define BIRD_HAND_NUM_OF_SERVOS	8

enum BIRD_HAND_MOTION_VARIANT {
	BIRD_HAND_SIGLE_STEP_POSTION_INCREMENT,
	BIRD_HAND_MACROSTEP_POSITION_INCREMENT,
	BIRD_HAND_MACROSTEP_ABSOLUTE_POSITION
};

struct bird_hand_command {
	int motion_steps;
	int ecp_query_step;
	BIRD_HAND_MOTION_VARIANT profile_type;
	double reciprocal_of_damping[BIRD_HAND_NUM_OF_SERVOS];
	double desired_torque[BIRD_HAND_NUM_OF_SERVOS];
	double desired_position[BIRD_HAND_NUM_OF_SERVOS];
};

struct bird_hand_status {
	double meassured_position[BIRD_HAND_NUM_OF_SERVOS];
	double meassured_torque[BIRD_HAND_NUM_OF_SERVOS];
	double meassured_current[BIRD_HAND_NUM_OF_SERVOS];
	bool upper_limit_of_absolute_value_of_desired_position_increment;
	bool upper_limit_of_absolute_value_of_computed_position_increment;
	bool upper_limit_of_absolute_position;
	bool lower_limit_of_absolute_position;
	bool upper_limit_of_absolute_value_of_desired_torque;
	bool lower_limit_of_absolute_value_of_desired_torque;
	bool upper_limit_of_absolute_value_of_meassured_torque;
	bool upper_limit_of_meassured_current;
};

struct bird_hand_configuration {
	int p_factor[BIRD_HAND_NUM_OF_SERVOS];
	int i_factor[BIRD_HAND_NUM_OF_SERVOS];
	int d_factor[BIRD_HAND_NUM_OF_SERVOS];
	int value_of_upper_limit_of_absolute_position;
	int value_of_lower_limit_of_absolute_position;
	int value_of_upper_limit_of_meassured_current;
	int value_of_upper_limit_of_absolute_value_of_torque;
	int value_of_lower_limit_of_absolute_value_of_torque;
	int value_of_lower_limit_of_absolute_value_of_meassured_torque;
	int value_of_upper_limit_of_position_increment;
};

enum BIRD_HAND_GEN_PROFILE {
	BIRD_HAND_TRAPEZOIDAL_VELOCITY,
	BIRD_HAND_CUBIC_POSITION,
	BIRD_HAND_GEN_PROFILE_NO_ACTION
};

#define BIRD_HAND_GEN_PARAMETERS_DATA_PORT "bird_hand_gen_paramteres_data_port"
#define BIRD_HAND_LOW_LEVEL_COMMAND_DATA_PORT "bird_hand_low_level_command_data_port"
#define BIRD_HAND_REPLY_DATA_REQUEST_PORT "bird_hand_reply_data_request_port"

struct single_controller_bird_hand_reply {
	double position;
	bool motion_in_progress;
}__attribute__((__packed__));

struct bird_hand_gen_parameters {
	BIRD_HAND_GEN_PROFILE profile_type;
	double dm[6];
	double aa[6];
	double da[6];
	double mv[6];
};

struct bird_hand_low_level_command {
	double em[6];
	double emdm[6];
	double aa[6];
	double da[6];
	double av[6];
	double tt;
	BIRD_HAND_GEN_PROFILE profile_type;
};

struct bird_hand_reply {
	single_controller_bird_hand_reply bird_hand_controller[6];
	bool contact;
};

}
}

#endif
