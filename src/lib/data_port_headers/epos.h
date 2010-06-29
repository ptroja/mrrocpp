/*
 **  EPOS.H
 */

#if !defined(__EPOS_DATA_PORT_H)
#define __EPOS_DATA_PORT_H

#define EPOS_DATA_PORT_SERVOS_NUMBER 7

namespace mrrocpp {
namespace lib {

enum EPOS_GEN_PROFILE
{
	TRAPEZOIDAL_VELOCITY, CUBIC_POSITION, EPOS_GEN_PROFILE_NO_ACTION
};

#define EPOS_GEN_PARAMETERS_DATA_PORT "epos_gen_paramteres_data_port"
#define EPOS_LOW_LEVEL_COMMAND_DATA_PORT "epos_low_level_command_data_port"
#define EPOS_REPLY_DATA_REQUEST_PORT "epos_reply_data_request_port"

struct single_controller_epos_reply
{
	double position;
	bool motion_in_progress;
}__attribute__((__packed__));

struct epos_gen_parameters
{
	EPOS_GEN_PROFILE profile_type;
	double dm[EPOS_DATA_PORT_SERVOS_NUMBER];
	double aa[EPOS_DATA_PORT_SERVOS_NUMBER];
	double da[EPOS_DATA_PORT_SERVOS_NUMBER];
	double mv[EPOS_DATA_PORT_SERVOS_NUMBER];
};

struct epos_low_level_command
{
	double em[EPOS_DATA_PORT_SERVOS_NUMBER];
	double emdm[EPOS_DATA_PORT_SERVOS_NUMBER];
	double aa[EPOS_DATA_PORT_SERVOS_NUMBER];
	double da[EPOS_DATA_PORT_SERVOS_NUMBER];
	double av[EPOS_DATA_PORT_SERVOS_NUMBER];
	double tt;
	EPOS_GEN_PROFILE profile_type;
};

struct epos_reply
{
	single_controller_epos_reply epos_controller[EPOS_DATA_PORT_SERVOS_NUMBER];
	bool contact;
};

}
}

#endif
