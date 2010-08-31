#if !defined(__SMB_DATA_PORT_H)
#define __SMB_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "robot/epos/dp_epos.h"

namespace mrrocpp {
namespace lib {
namespace smb {

const int DATA_PORT_SERVOS_NUMBER = 2;
const int DATA_PORT_LEG_CLAMP_NUMBER = 3;

struct mp_to_ecp_parameters
{
	int locking_device_clamp_number;
	epos::EPOS_GEN_PROFILE motion_type;
	epos::smb_mp_to_ecp_cubic_trapezoidal_parameters cubic_trapezoidal[DATA_PORT_SERVOS_NUMBER];
};

struct leg_reply
{
	bool is_inserted;
	bool is_locked;
	bool insertion_in_progress;
	bool locking_in_progress;
}__attribute__((__packed__));

enum PIN_INSERTION
{
	INSERT, WITHDRAWN, PIN_INSERTION_NO_ACTION
}; // namespace mrrocpp

enum PIN_LOCKING
{
	CLAMB, UNCLAMB, SMB_PIN_LOCKING_NO_ACTION
}; // namespace mrrocpp

const std::string MULTI_PIN_INSERTION_DATA_PORT = "SMB_MULTI_PIN_INSERTION_DATA_PORT";
const std::string MULTI_PIN_LOCKING_DATA_PORT = "SMB_MULTI_PIN_LOCKING_DATA_PORT";
const std::string MULTI_LEG_REPLY_DATA_REQUEST_PORT = "SMB_MULTI_LEG_REPLY_DATA_REQUEST_PORT";

struct multi_pin_insertion_td
{
	PIN_INSERTION leg[DATA_PORT_LEG_CLAMP_NUMBER];
};

struct multi_pin_locking_td
{
	PIN_LOCKING leg[DATA_PORT_LEG_CLAMP_NUMBER];
};

struct multi_leg_reply_td
{
	leg_reply leg[DATA_PORT_LEG_CLAMP_NUMBER];
};

} // namespace smb
}
}

#endif
