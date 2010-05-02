#if !defined(_SMB_CONST_H)
#define _SMB_CONST_H

#include "lib/data_port_headers/smb.h"
#include "lib/data_port_headers/epos.h"

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {

struct smb_cbuffer {
	double em[4];
	double emdm[4];
	double aa[4];
	double da[4];
	double av[4];
	double tt;
	lib::EPOS_GEN_PROFILE profile_type;
	lib::SMB_PIN_INSERTION pin_insertion[3];
	lib::SMB_PIN_LOCKING pin_locking[3];
};

struct smb_rbuffer {
	smb_leg_reply leg[3];
	single_controller_epos_reply epos_controller[4];
};

#define EDP_SMB_SECTION "[edp_smb]"
#define ECP_SMB_SECTION "[ecp_smb]"

#define SMB_NUM_OF_SERVOS	4

} // namespace lib
} // namespace mrrocpp

#endif /* _SMB_CONST_H */
