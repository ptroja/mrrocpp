/*
 **  SMB.H
 */

#if !defined(__SMB_DATA_PORT_H)
#define __SMB_DATA_PORT_H

#define SMB_DATA_PORT_SERVOS_NUMBER 2
#define SMB_DATA_PORT_LEG_CLAMP_NUMBER 3

#include "epos.h"

namespace mrrocpp {
namespace lib {

struct smb_mp_to_ecp_parameters
{
	int locking_device_clamp_number;
	EPOS_GEN_PROFILE motion_type;
	smb_mp_to_ecp_cubic_spline_parameters cubic_spline[SMB_DATA_PORT_SERVOS_NUMBER];
};

struct smb_leg_reply
{
	bool is_inserted;
	bool is_locked;
	bool insertion_in_progress;
	bool locking_in_progress;
}__attribute__((__packed__));

enum SMB_PIN_INSERTION
{
	INSERT, WITHDRAWN, SMB_PIN_INSERTION_NO_ACTION
}; // namespace mrrocpp

enum SMB_PIN_LOCKING
{
	CLAMB, UNCLAMB, SMB_PIN_LOCKING_NO_ACTION
}; // namespace mrrocpp

#define SMB_MULTI_PIN_INSERTION_DATA_PORT "smb_multi_pin_insertion_data_port"
#define SMB_MULTI_PIN_LOCKING_DATA_PORT "smb_multi_pin_locking_data_port"
#define SMB_MULTI_LEG_REPLY_DATA_REQUEST_PORT "smb_multi_leg_reply_data_request_port"

struct smb_multi_pin_insertion
{
	SMB_PIN_INSERTION leg[SMB_DATA_PORT_LEG_CLAMP_NUMBER];
};

struct smb_multi_pin_locking
{
	SMB_PIN_LOCKING leg[SMB_DATA_PORT_LEG_CLAMP_NUMBER];
};

struct smb_multi_leg_reply
{
	smb_leg_reply leg[SMB_DATA_PORT_LEG_CLAMP_NUMBER];
};

}
}

#endif
