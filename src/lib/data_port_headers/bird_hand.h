/*
 **  BIRD_HAND.H
 */

#if !defined(__BIRD_HAND_DATA_PORT_H)
#define __BIRD_HAND_DATA_PORT_H

namespace mrrocpp {
namespace lib {

#define BIRD_HAND_COMMAND_DATA_PORT "bird_hand_configuration_data_port"

#define BIRD_HAND_CONFIGURATION_DATA_PORT "bird_hand_configuration_data_port"
#define BIRD_HAND_CONFIGURATION_DATA_REQUEST_PORT "bird_hand_configuration_data_request_port"

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
