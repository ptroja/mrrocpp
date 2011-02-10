#if !defined(_SPKM_CONST_H)
#define _SPKM_CONST_H

/*!
 * @file
 * @brief File contains constants and structures for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

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
	CBUFFER_EPOS_POSE,
	CBUFFER_EPOS_BRAKE_COMMAND,
	CBUFFER_EPOS_CLEAR_FAULT
};

//! Pose specification variants
typedef enum _POSE_SPECIFICATION
{
	FRAME, JOINT, MOTOR
} POSE_SPECIFICATION;

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP command buffer
 * @ingroup spkm
 */
struct cbuffer
{
	//! Variant of the command
	CBUFFER_VARIANT variant;

	//! Pose specification type
	POSE_SPECIFICATION pose_specification;

	epos::epos_simple_command epos_simple_command_structure;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & variant;
		switch (variant) {
			case CBUFFER_EPOS_POSE:
				ar & pose_specification;
				ar & epos_simple_command_structure;
				break;
			default:
				break;
		};
	}
}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP reply buffer
 * @ingroup spkm
 */
struct rbuffer
{
	lib::frame_tab current_frame;
	epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];
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
}__attribute__((__packed__));

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
