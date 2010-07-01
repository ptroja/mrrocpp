#if !defined(_SMB_CONST_H)
#define _SMB_CONST_H

#include "lib/data_port_headers/smb.h"
#include "lib/data_port_headers/epos.h"

#include "lib/impconst.h"

namespace mrrocpp {
namespace lib {

const robot_name_t ROBOT_SMB = "ROBOT_SMB";

enum SMB_CBUFFER_VARIANT
{
	SMB_CBUFFER_EPOS_CUBIC_COMMAND,
	SMB_CBUFFER_EPOS_TRAPEZOIDAL_COMMAND,
	SMB_CBUFFER_PIN_INSERTION,
	SMB_CBUFFER_PIN_LOCKING
};

struct smb_cbuffer
{
	SMB_CBUFFER_VARIANT variant;
	union
	{
		epos_cubic_command epos_cubic_command_structure;
		epos_trapezoidal_command epos_trapezoidal_command_structure;
		epos_operational_command epos_operational_command_structure;
		smb_multi_pin_insertion multi_pin_insertion;
		smb_multi_pin_locking multi_pin_locking;
	};
};

#define SMB_NUM_OF_SERVOS	4

struct smb_rbuffer
{
	smb_multi_leg_reply multi_leg_reply;
	single_controller_epos_reply epos_controller[SMB_NUM_OF_SERVOS];
};

#define EDP_SMB_SECTION "[edp_smb]"
#define ECP_SMB_SECTION "[ecp_smb]"

} // namespace lib
} // namespace mrrocpp

#endif /* _SMB_CONST_H */
