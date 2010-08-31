#if !defined(__SMB_DATA_PORT_H)
#define __SMB_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#define SMB_DATA_PORT_SERVOS_NUMBER 2
#define SMB_DATA_PORT_LEG_CLAMP_NUMBER 3

#include "robot/epos/dp_epos.h"

namespace mrrocpp {
namespace lib {

struct smb_mp_to_ecp_parameters
{
	int locking_device_clamp_number;
	epos::EPOS_GEN_PROFILE motion_type;
	epos::smb_mp_to_ecp_cubic_trapezoidal_parameters cubic_trapezoidal[SMB_DATA_PORT_SERVOS_NUMBER];
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

const std::string SMB_MULTI_PIN_INSERTION_DATA_PORT = "SMB_MULTI_PIN_INSERTION_DATA_PORT";
const std::string SMB_MULTI_PIN_LOCKING_DATA_PORT = "SMB_MULTI_PIN_LOCKING_DATA_PORT";
const std::string SMB_MULTI_LEG_REPLY_DATA_REQUEST_PORT = "SMB_MULTI_LEG_REPLY_DATA_REQUEST_PORT";

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
