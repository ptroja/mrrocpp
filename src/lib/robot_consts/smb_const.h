#if !defined(_SMB_CONST_H)
#define _SMB_CONST_H

#include "lib/swarmitfix.h"

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {

struct smb_cbuffer {
	double em[3];
	double emdm[3];
	double aa[3];
	double da[3];
	double av[3];
	double tt;
	lib::EPOS_GEN_PROFILE profile_type;
	lib::SMB_PIN_INSERTION pin_insertion[2];
	lib::SMB_PIN_LOCKING pin_locking[2];
};

struct smb_rbuffer {
	double position[3];
	bool motion_in_progress[3];
	bool pin_inserted[2];
	bool pin_clamped[2];
};

#define EDP_SMB_SECTION "[edp_smb]"
#define ECP_SMB_SECTION "[ecp_smb]"

#define SMB_NUM_OF_SERVOS	2

} // namespace lib
} // namespace mrrocpp

#endif /* _SMB_CONST_H */
