/*
 **  SPKM.H
 */

#if !defined(__SPKM_DATA_PORT_H)
#define __SPKM_DATA_PORT_H

#define SPKM_DATA_PORT_SERVOS_NUMBER 7

#include "epos.h"

namespace mrrocpp {
namespace lib {

struct smb_mp_to_ecp_parameters
{
	EPOS_GEN_PROFILE motion_type;
	smb_mp_to_ecp_cubic_spline_parameters cubic_spline[SMB_DATA_PORT_SERVOS_NUMBER];
	lib::frame_tab goal_frame;
	double m;
	double tau;
	bool guarded_move_manipulator;
};

}
}

#endif
