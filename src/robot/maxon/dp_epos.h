#if !defined(__EPOS_DATA_PORT_H)
#define __EPOS_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for processes cooperating with Epos2 Maxon controllers
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup epos
 */

#include <boost/serialization/serialization.hpp>
#include <string>

#include"base/lib/impconst.h"

#include "base/lib/mrmath/homog_matrix.h"

namespace mrrocpp {
namespace lib {
namespace epos {

/*!
 * @brief SwarmItFix Epos total number of servos
 * @ingroup epos
 */
static const int EPOS_DATA_PORT_SERVOS_NUMBER = 7; // nie moze byc mniej niz 6 bo wykorzystywane takze do external

/*!
 * @brief SwarmItFix Epos generator possible profiles enum
 * @ingroup epos
 */
enum EPOS_GEN_PROFILE
{
	TRAPEZOIDAL_VELOCITY, CUBIC_POSITION, OPERATIONAL_SPACE, EPOS_GEN_PROFILE_NO_ACTION
};

/*!
 * @brief SwarmItFix Epos motion variant
 * @ingroup epos
 */
enum EPOS_MOTION_VARIANT
{
	NON_SYNC_TRAPEZOIDAL, SYNC_TRAPEZOIDAL, SYNC_POLYNOMIAL, OPERATIONAL
};

/*!
 * @brief SwarmItFix Epos simple motor command data port
 * @ingroup epos
 */
const std::string EPOS_MOTOR_COMMAND_DATA_PORT = "EPOS_MOTOR_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos simple joint command data port
 * @ingroup epos
 */
const std::string EPOS_JOINT_COMMAND_DATA_PORT = "EPOS_JOINT_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos simple external command data port
 * @ingroup epos
 */
const std::string EPOS_EXTERNAL_COMMAND_DATA_PORT = "EPOS_EXTERNAL_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos cubic trajectory command data port - deprecated
 * @ingroup epos
 */
const std::string EPOS_CUBIC_COMMAND_DATA_PORT = "EPOS_CUBIC_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos trapezoidal trajectory command data port - deprecated
 * @ingroup epos
 */
const std::string EPOS_TRAPEZOIDAL_COMMAND_DATA_PORT = "EPOS_TRAPEZOIDAL_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos operational space trajectory command data port - deprecated
 * @ingroup epos
 */
const std::string EPOS_OPERATIONAL_COMMAND_DATA_PORT = "EPOS_OPERATIONAL_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos motor brake command data port
 * @ingroup epos
 */
const std::string EPOS_BRAKE_COMMAND_DATA_PORT = "EPOS_BRAKE_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos clear fault command data port
 * @ingroup epos
 *
 */
const std::string EPOS_CLEAR_FAULT_DATA_PORT = "EPOS_CLEAR_FAULT_DATA_PORT";

/*!
 * @brief SwarmItFix Epos status data request port
 * @ingroup epos
 */
const std::string EPOS_MOTOR_REPLY_DATA_REQUEST_PORT = "EPOS_MOTOR_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Epos status data request port
 * @ingroup epos
 */
const std::string EPOS_JOINT_REPLY_DATA_REQUEST_PORT = "EPOS_JOINT_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Epos status data request port
 * @ingroup epos
 */
const std::string EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT = "EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Epos controller mp to ecp command
 * @ingroup epos
 */
struct mp_to_ecp_cubic_trapezoidal_parameters
{
	double dm;
	double aa;
	double da;
	double mv;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & dm;
		ar & aa;
		ar & da;
		ar & mv;
	}
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Epos single controller status
 * @ingroup epos
 */
struct single_controller_epos_reply
{
	int16_t current;
	double position;
	bool motion_in_progress;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & current;
		ar & position;
		ar & motion_in_progress;
	}
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Epos cubic trajectory command - deprecated
 * @ingroup epos
 */
struct epos_trapezoidal_command
{
	double emdm[EPOS_DATA_PORT_SERVOS_NUMBER];
	double aa[EPOS_DATA_PORT_SERVOS_NUMBER];
	double da[EPOS_DATA_PORT_SERVOS_NUMBER];
	double av[EPOS_DATA_PORT_SERVOS_NUMBER];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & emdm;
		ar & aa;
		ar & da;
		ar & av;
	}
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Epos motor and joint command, called from UI
 * @ingroup epos
 */
struct epos_simple_command
{
	EPOS_MOTION_VARIANT motion_variant;
	double desired_position[EPOS_DATA_PORT_SERVOS_NUMBER];
	double estimated_time;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & motion_variant;
		ar & desired_position;
		ar & estimated_time;
	}
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Epos trapezoidal trajectory command - deprecated
 * @ingroup epos
 */
struct epos_cubic_command
{
	double em[EPOS_DATA_PORT_SERVOS_NUMBER];
	double emdm[EPOS_DATA_PORT_SERVOS_NUMBER];
	double tt;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & em;
		ar & emdm;
		ar & tt;
	}
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Epos operational trajectory command data port - deprecated
 * @ingroup epos
 */
struct epos_operational_command
{
	double em[EPOS_DATA_PORT_SERVOS_NUMBER];
	double v[EPOS_DATA_PORT_SERVOS_NUMBER];
	double tau;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & em;
		ar & v;
		ar & tau;
	}
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Epos all controllers status
 * @ingroup epos
 */
struct epos_reply
{
	lib::Homog_matrix current_frame;
	single_controller_epos_reply epos_controller[EPOS_DATA_PORT_SERVOS_NUMBER];
	bool contact;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & current_frame;
		ar & epos_controller;
		ar & contact;
	}
};

} // namespace epos
} // namespace lib
} // namespace mrrocpp

#endif
