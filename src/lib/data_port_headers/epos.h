/*
 **  EPOS.H
 */

#if !defined(__EPOS_DATA_PORT_H)
#define __EPOS_DATA_PORT_H

namespace mrrocpp {
namespace lib {

#define EPOS_COMMAND_DATA_PORT "epos_command_data_port"
#define EPOS_REPLY_DATA_REQUEST_PORT "epos_reply_data_request_port"

struct epos_command {
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
