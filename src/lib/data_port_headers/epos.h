/*
 **  EPOS.H
 */

#if !defined(__EPOS_DATA_PORT_H)
#define __EPOS_DATA_PORT_H

namespace mrrocpp {
namespace lib {

enum EPOS_GEN_PROFILE {
	TRAPEZOIDAL_VELOCITY, CUBIC_POSITION, EPOS_GEN_PROFILE_NO_ACTION
};

#define EPOS_GEN_PARAMETERS_DATA_PORT "epos_gen_paramteres_data_port"
#define EPOS_LOW_LEVEL_COMMAND_DATA_PORT "epos_low_level_command_data_port"
#define EPOS_REPLY_DATA_REQUEST_PORT "epos_reply_data_request_port"


struct epos_gen_parameters {
	EPOS_GEN_PROFILE profile_type;
	double dm[6];
	double aa[6];
	double da[6];
	double mv[6];
};

struct epos_low_level_command {
	double em[6];
	double emdm[6];
	double aa[6];
	double da[6];
	double av[6];
	double tt;
	EPOS_GEN_PROFILE profile_type;
};

struct epos_reply {
	double position[6];
	bool motion_in_progress[6];
};

}
}

#endif
