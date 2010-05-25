#if !defined(_SMB_CONST_H)
#define _SMB_CONST_H

#include "lib/data_port_headers/smb.h"
#include "lib/data_port_headers/epos.h"

namespace mrrocpp {

}

using namespace mrrocpp;

namespace mrrocpp {
namespace lib {

enum SMB_CBUFFER_VARIANT {
	SMB_CBUFFER_EPOS_LOW_LEVEL_COMMAND,
	SMB_CBUFFER_EPOS_GEN_PARAMETERS,
	SMB_CBUFFER_PIN_INSERTION,
	SMB_CBUFFER_PIN_LOCKING
};

struct smb_cbuffer {
	SMB_CBUFFER_VARIANT variant;
	union {
		epos_low_level_command epos_low_level_command_structure;
		epos_gen_parameters epos_gen_parameters_structure;
		smb_multi_pin_insertion multi_pin_insertion;
		smb_multi_pin_locking multi_pin_locking;
	};
};

#define SMB_NUM_OF_SERVOS	4

struct smb_rbuffer {
	smb_multi_leg_reply multi_leg_reply;
	single_controller_epos_reply epos_controller[SMB_NUM_OF_SERVOS];
};

#define EDP_SMB_SECTION "[edp_smb]"
#define ECP_SMB_SECTION "[ecp_smb]"

} // namespace lib
} // namespace mrrocpp

#endif /* _SMB_CONST_H */
