#if !defined(_SPKM_CONST_H)
#define _SPKM_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include <ostream>
#include <exception>

#include "robot/spkm/dp_spkm.h"

namespace mrrocpp {
namespace lib {
namespace spkm {

/*!
 * @brief SwarmItFix Parallel Kinematic Machine robot label
 * @ingroup spkm
 */
const robot_name_t ROBOT_NAME = "ROBOT_SPKM";

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP command buffer variant enum
 * @ingroup spkm
 */
enum CBUFFER_VARIANT
{
	CBUFFER_EPOS_MOTOR_COMMAND,
	CBUFFER_EPOS_JOINT_COMMAND,
	CBUFFER_EPOS_EXTERNAL_COMMAND,
	CBUFFER_EPOS_CUBIC_COMMAND,
	CBUFFER_EPOS_TRAPEZOIDAL_COMMAND,
	CBUFFER_EPOS_OPERATIONAL_COMMAND,
	CBUFFER_EPOS_BRAKE_COMMAND
};

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP command buffer
 * @ingroup spkm
 */
struct cbuffer
{
	CBUFFER_VARIANT variant;
	union
	{
		epos::epos_cubic_command epos_cubic_command_structure;
		epos::epos_simple_command epos_simple_command_structure;
		epos::epos_trapezoidal_command epos_trapezoidal_command_structure;
		epos::epos_operational_command epos_operational_command_structure;
	};

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & variant;
		switch (variant)
		{
			case CBUFFER_EPOS_MOTOR_COMMAND:
			case CBUFFER_EPOS_JOINT_COMMAND:
			case CBUFFER_EPOS_EXTERNAL_COMMAND:
				ar & epos_simple_command_structure;
				break;
			case CBUFFER_EPOS_CUBIC_COMMAND:
				ar & epos_cubic_command_structure;
				break;
			case CBUFFER_EPOS_TRAPEZOIDAL_COMMAND:
				ar & epos_trapezoidal_command_structure;
				break;
			case CBUFFER_EPOS_OPERATIONAL_COMMAND:
				ar & epos_operational_command_structure;
				break;
			case CBUFFER_EPOS_BRAKE_COMMAND:
				break;
			default:
				throw std::bad_cast("unknown SPKM CBUFFER_VARIANT");
		}
	}

	friend std::ostream& operator<<(std::ostream& os, const cbuffer& m)
	{
		switch (m.variant)
		{
			case CBUFFER_EPOS_MOTOR_COMMAND:
				os << "CBUFFER_EPOS_MOTOR_COMMAND:\n";
				break;
			case CBUFFER_EPOS_JOINT_COMMAND:
				os << "CBUFFER_EPOS_JOINT_COMMAND:\n";
				break;
			case CBUFFER_EPOS_EXTERNAL_COMMAND:
				os << "CBUFFER_EPOS_EXTERNAL_COMMAND:\n";
				break;
			case CBUFFER_EPOS_CUBIC_COMMAND:
				os << "CBUFFER_EPOS_CUBIC_COMMAND:\n";
				for (int i = 0; i < lib::epos::EPOS_DATA_PORT_SERVOS_NUMBER; ++i) {
					os << "\t" << m.epos_cubic_command_structure.aa[i] << "\t" << m.epos_cubic_command_structure.av[i]
							<< "\t" << m.epos_cubic_command_structure.da[i] << "\t"
							<< m.epos_cubic_command_structure.emdm[i] << "\n";
				}
				break;
			case CBUFFER_EPOS_TRAPEZOIDAL_COMMAND:
				os << "CBUFFER_EPOS_TRAPEZOIDAL_COMMAND:\n";
				for (int i = 0; i < lib::epos::EPOS_DATA_PORT_SERVOS_NUMBER; ++i) {
					os << "\t" << m.epos_trapezoidal_command_structure.em[i] << "\t"
							<< m.epos_trapezoidal_command_structure.emdm[i] << "\n";
				}
				os << "\t" << m.epos_trapezoidal_command_structure.tt << "\n";
				break;
			case CBUFFER_EPOS_OPERATIONAL_COMMAND:
				os << "CBUFFER_EPOS_OPERATIONAL_COMMAND:\n";
				for (int i = 0; i < lib::epos::EPOS_DATA_PORT_SERVOS_NUMBER; ++i) {
					os << "\t" << m.epos_operational_command_structure.em[i] << "\t"
							<< m.epos_operational_command_structure.v[i] << "\n";
				}
				os << "\t" << m.epos_operational_command_structure.tau << "\n";
				break;
			case CBUFFER_EPOS_BRAKE_COMMAND:
				os << "CBUFFER_EPOS_BRAKE_COMMAND\n";
				break;
			default:
				os << "Error: unknown CBUFFER_VARIANT";
		}
		return os;
	}
};

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP reply buffer
 * @ingroup spkm
 */
struct rbuffer
{
	epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];
	bool contact;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & epos_controller;
		ar & contact;
	}
};

/*!
 * @brief configuration file EDP SwarmItFix Parallel Kinematic Machine section string
 * @ingroup spkm
 */
const std::string EDP_SECTION = "[edp_spkm]";

/*!
 * @brief configuration file ECP SwarmItFix Parallel Kinematic Machine section string
 * @ingroup spkm
 */
const std::string ECP_SECTION = "[ecp_spkm]";

} // namespace spkm
} // namespace lib
} // namespace mrrocpp

#endif /* _SPKM_CONST_H */
