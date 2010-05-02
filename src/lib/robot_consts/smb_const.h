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
	MB_CBUFFER_PIN_LOCKING
};

struct smb_cbuffer {
	SMB_CBUFFER_VARIANT variant;
	union {
		epos_low_level_command epos_data_port_command_structure;
		epos_gen_parameters epos_data_port_gen_parameters_structure;
		lib::SMB_PIN_INSERTION pin_insertion[3];
		lib::SMB_PIN_LOCKING pin_locking[3];
	};
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
