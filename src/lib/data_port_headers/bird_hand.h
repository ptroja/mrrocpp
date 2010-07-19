/*
 **  BIRD_HAND.H
 */

#if !defined(__BIRD_HAND_DATA_PORT_H)
#define __BIRD_HAND_DATA_PORT_H

namespace mrrocpp {
namespace lib {

// ponizej konieczne zdefiniowanie typu 64bitowego bo inaczej przepalnia sie typ 32bitowy przy mnozeniu
#define BIRD_HAND_STEP_TIME_IN_NS ((uint64_t) 2000000)

#define BIRD_HAND_COMMAND_DATA_PORT "bird_hand_command_data_port"
#define BIRD_HAND_STATUS_DATA_REQUEST_PORT "bird_hand_status_data_request_port"

#define BIRD_HAND_CONFIGURATION_DATA_PORT "bird_hand_configuration_data_port"
#define BIRD_HAND_CONFIGURATION_DATA_REQUEST_PORT "bird_hand_configuration_data_request_port"

#define BIRD_HAND_NUM_OF_SERVOS	8
#define BIRD_HAND_THUMB_F_NUM_OF_SERVOS	2
#define BIRD_HAND_INDEX_F_NUM_OF_SERVOS	3
#define BIRD_HAND_RING_F_NUM_OF_SERVOS	3

enum BIRD_HAND_MOTION_VARIANT {
	BIRD_HAND_SIGLE_STEP_POSTION_INCREMENT = 0,
	BIRD_HAND_MACROSTEP_POSITION_INCREMENT = 1,
	BIRD_HAND_MACROSTEP_ABSOLUTE_POSITION = 2
};

struct bird_hand_single_joint_command {
	BIRD_HAND_MOTION_VARIANT profile_type;
	double reciprocal_of_damping;
	double desired_torque;
	double desired_position;
};

struct bird_hand_single_joint_status {
	double meassured_position;
	double meassured_torque;
	double meassured_current;
	bool upper_limit_of_absolute_value_of_desired_position_increment;
	bool upper_limit_of_absolute_value_of_computed_position_increment;
	bool upper_limit_of_absolute_position;
	bool lower_limit_of_absolute_position;
	bool upper_limit_of_absolute_value_of_desired_torque;
	bool lower_limit_of_absolute_value_of_desired_torque;
	bool upper_limit_of_absolute_value_of_meassured_torque;
	bool upper_limit_of_meassured_current;
};

struct bird_hand_single_joint_configuration {
	int p_factor;
	int i_factor;
	int d_factor;
	int value_of_upper_limit_of_absolute_position;
	int value_of_lower_limit_of_absolute_position;
	int value_of_upper_limit_of_meassured_current;
	int value_of_upper_limit_of_absolute_value_of_torque;
	int value_of_lower_limit_of_absolute_value_of_torque;
	int value_of_lower_limit_of_absolute_value_of_meassured_torque;
	int value_of_upper_limit_of_position_increment;
};

struct bird_hand_command {
	int motion_steps;
	int ecp_query_step;
	bird_hand_single_joint_command thumb_f[BIRD_HAND_THUMB_F_NUM_OF_SERVOS];
	bird_hand_single_joint_command index_f[BIRD_HAND_INDEX_F_NUM_OF_SERVOS];
	bird_hand_single_joint_command ring_f[BIRD_HAND_RING_F_NUM_OF_SERVOS];
};

struct bird_hand_status {
	bird_hand_single_joint_status thumb_f[BIRD_HAND_THUMB_F_NUM_OF_SERVOS];
	bird_hand_single_joint_status index_f[BIRD_HAND_INDEX_F_NUM_OF_SERVOS];
	bird_hand_single_joint_status ring_f[BIRD_HAND_RING_F_NUM_OF_SERVOS];
};

struct bird_hand_configuration {
	bird_hand_single_joint_configuration
			thumb_f[BIRD_HAND_THUMB_F_NUM_OF_SERVOS];
	bird_hand_single_joint_configuration
			index_f[BIRD_HAND_INDEX_F_NUM_OF_SERVOS];
	bird_hand_single_joint_configuration ring_f[BIRD_HAND_RING_F_NUM_OF_SERVOS];
};

}
}

#endif
