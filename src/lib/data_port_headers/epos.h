/*
 **  EPOS.H
 */

#if !defined(__EPOS_DATA_PORT_H)
#define __EPOS_DATA_PORT_H

#include <string>

#define EPOS_DATA_PORT_SERVOS_NUMBER 7

namespace mrrocpp {
namespace lib {

enum EPOS_GEN_PROFILE
{
	TRAPEZOIDAL_VELOCITY, CUBIC_POSITION, OPERATIONAL_SPACE, EPOS_GEN_PROFILE_NO_ACTION
};

const std::string EPOS_GEN_PARAMETERS_DATA_PORT = "epos_gen_paramteres_data_port";
const std::string EPOS_LOW_LEVEL_COMMAND_DATA_PORT = "epos_low_level_command_data_port";
const std::string EPOS_REPLY_DATA_REQUEST_PORT = "epos_reply_data_request_port";

struct smb_mp_to_ecp_cubic_spline_parameters
{
	double dm;
	double aa;
	double da;
	double mv;
};

struct single_controller_epos_reply
{
	double position;
	bool motion_in_progress;
	bool buffer_full;
}__attribute__((__packed__));

struct epos_cubic_command
{
	double emdm[EPOS_DATA_PORT_SERVOS_NUMBER];
	double aa[EPOS_DATA_PORT_SERVOS_NUMBER];
	double da[EPOS_DATA_PORT_SERVOS_NUMBER];
	double av[EPOS_DATA_PORT_SERVOS_NUMBER];
};

struct epos_spline_command
{
	double em[EPOS_DATA_PORT_SERVOS_NUMBER];
	double emdm[EPOS_DATA_PORT_SERVOS_NUMBER];
	double tt;
};

struct epos_operational_command
{
	double em[EPOS_DATA_PORT_SERVOS_NUMBER];
	double v[EPOS_DATA_PORT_SERVOS_NUMBER];
	double tau;
};

struct epos_reply
{
	single_controller_epos_reply epos_controller[EPOS_DATA_PORT_SERVOS_NUMBER];
	bool contact;
};

}
}

#endif
