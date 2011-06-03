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

#include "base/lib/mrmath/homog_matrix.h"

namespace mrrocpp {
namespace lib {
namespace spkm {

/*!
 * @brief SwarmItFix Parallel Kinematic Machine robot label
 * @ingroup spkm
 */
const robot_name_t ROBOT_NAME = "spkm";

/*!
 * @brief SwarmItFix Parallel Kinematic Machine number of motors.
 *
 * The kinematics, as well as control of the whole PKM, is solved for 6DOF - three for PM and three for SW .
 *
 * @ingroup spkm
 */
const int NUM_OF_SERVOS = 6;

/*!
 * @brief Number of segments making up the whole PKM motion.
 *
 *
 * @author tkornuta
 * @ingroup spkm
 */
const unsigned int NUM_OF_MOTION_SEGMENTS = 64;

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP command buffer variant enum
 * @ingroup spkm
 */
enum CBUFFER_VARIANT
{
	POSE, QUICKSTOP, CLEAR_FAULT
};

/*!
 * Pose specification variants
 * @ingroup spkm
 */
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

	//! Motion interpolation variant
	lib::epos::EPOS_MOTION_VARIANT motion_variant;

	double estimated_time;

	int32_t motor_pos[NUM_OF_SERVOS];
	double joint_pos[NUM_OF_SERVOS];
	double goal_pos[6];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & variant;
		switch (variant)
		{
			case POSE:
				ar & pose_specification;
				switch (pose_specification)
				{
					case FRAME:
						ar & goal_pos;
						break;
					case JOINT:
						ar & joint_pos;
						break;
					case MOTOR:
						ar & motor_pos;
						break;
				}
				ar & motion_variant;
				ar & estimated_time;
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
	lib::Homog_matrix current_frame;
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
};

} // namespace spkm
} // namespace lib
} // namespace mrrocpp

#endif /* _SPKM_CONST_H */
