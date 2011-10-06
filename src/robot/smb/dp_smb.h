#if !defined(__SMB_DATA_PORT_H)
#define __SMB_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include "robot/maxon/dp_epos.h"

namespace mrrocpp {
namespace lib {
namespace smb {

/*!
 * @brief SwarmItFix Mobile Base multi pin insertion command data port
 * @ingroup smb
 */
const std::string MULTI_PIN_INSERTION_DATA_PORT = "smb_festo_command_data_port";

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
	bool is_up;
	bool is_down;
	bool is_attached;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & is_up;
		ar & is_down;
		ar & is_attached;
	}

}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Mobile Base single leg festo command
 * @ingroup smb
 */
enum FESTO_LEG
{
	UP, DOWN
};
// namespace mrrocpp

// namespace mrrocpp

/*!
 * @brief SwarmItFix Mobile Base multi pin insertion command
 * @ingroup smb
 */
struct festo_command_td
{
	FESTO_LEG leg[LEG_CLAMP_NUMBER];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & leg;
	}

}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Mobile Base multi leg reply
 * @ingroup smb
 */
struct multi_leg_reply_td
{
	leg_reply leg[LEG_CLAMP_NUMBER];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & leg;
	}

}__attribute__((__packed__));

} // namespace smb
}
}

#endif
