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

/*!
 * @brief SwarmItFix Mobile Base multi pin insertion command data port
 * @ingroup smb
 */
const std::string MULTI_PIN_INSERTION_DATA_PORT = "SMB_MULTI_PIN_INSERTION_DATA_PORT";

/*!
 * @brief SwarmItFix Mobile Base mulri pin locking command data port
 * @ingroup smb
 */
const std::string MULTI_PIN_LOCKING_DATA_PORT = "SMB_MULTI_PIN_LOCKING_DATA_PORT";

/*!
 * @brief SwarmItFix Mobile Base status data request port
 * @ingroup smb
 */
const std::string MULTI_LEG_REPLY_DATA_REQUEST_PORT = "SMB_MULTI_LEG_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Mobile Base total number of servos
 * @ingroup smb
 */
const int NUM_OF_SERVOS = 2;

/*!
 * @brief SwarmItFix Mobile Base total number of legs
 * @ingroup smb
 */
const int LEG_CLAMP_NUMBER = 3;

/*!
 * @brief SwarmItFix Mobile Base mp to ecp command
 * @ingroup smb
 */
struct mp_to_ecp_parameters
{
	int locking_device_clamp_number;
	epos::EPOS_GEN_PROFILE motion_type;
	epos::mp_to_ecp_cubic_trapezoidal_parameters cubic_trapezoidal[NUM_OF_SERVOS];
};

/*!
 * @brief SwarmItFix Mobile Base single leg status
 * @ingroup smb
 */
struct leg_reply
{
	bool is_inserted;
	bool is_locked;
	bool insertion_in_progress;
	bool locking_in_progress;
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Mobile Base pin insertion command
 * @ingroup smb
 */
enum PIN_INSERTION
{
	INSERT, WITHDRAWN, PIN_INSERTION_NO_ACTION
}; // namespace mrrocpp

/*!
 * @brief SwarmItFix Mobile Base pin locking command
 * @ingroup smb
 */
enum PIN_LOCKING
{
	CLAMB, UNCLAMB, SMB_PIN_LOCKING_NO_ACTION
}; // namespace mrrocpp

/*!
 * @brief SwarmItFix Mobile Base multi pin insertion command
 * @ingroup smb
 */
struct multi_pin_insertion_td
{
	PIN_INSERTION leg[LEG_CLAMP_NUMBER];
};

/*!
 * @brief SwarmItFix Mobile Base multi pin locking command
 * @ingroup smb
 */
struct multi_pin_locking_td
{
	PIN_LOCKING leg[LEG_CLAMP_NUMBER];
};

/*!
 * @brief SwarmItFix Mobile Base multi leg reply
 * @ingroup smb
 */
struct multi_leg_reply_td
{
	leg_reply leg[LEG_CLAMP_NUMBER];
};

} // namespace smb
}
}

#endif
