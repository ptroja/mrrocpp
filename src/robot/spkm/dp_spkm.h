#if !defined(__SPKM_DATA_PORT_H)
#define __SPKM_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include <boost/serialization/serialization.hpp>

#include "base/lib/mrmath/homog_matrix.h"
#include "robot/maxon/dp_epos.h"

#include "const_spkm.h"

namespace mrrocpp {
namespace lib {
namespace spkm {

/*!
 * @brief SwarmItFix Parallel Kinematic Machine mp to ecp command
 * @ingroup spkm
 */
typedef struct _segment
{
	//! The goal pose of the manipulator
	lib::Homog_matrix goal_pose;

	//! Interpolation type for the motion
	lib::epos::EPOS_MOTION_VARIANT motion_type;

	//! Allowed time for the motion in seconds.
	//! If 0, then the time will be limited by the motor limits.
	//! If > 0 and greater than a limit imposed by the motors, then the motion will be slowed down.
	//! In another case, the NACK will be replied.
	double duration;

	//! True if the contact is expected during the motion.
	//! The NACK will be replied if:
	//! - the contact was expected and did not happened
	//! - OR the contact was NOT expected and did happened.
	bool guarded_motion;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(goal_pose);
		ar & BOOST_SERIALIZATION_NVP(motion_type);
		ar & BOOST_SERIALIZATION_NVP(duration);
		ar & BOOST_SERIALIZATION_NVP(guarded_motion);
	}
} segment_t;

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
	POSE_SPECIFICATION set_pose_specification;

	//! Pose specification type
	POSE_SPECIFICATION get_pose_specification;

	//! Motion interpolation variant
	lib::epos::EPOS_MOTION_VARIANT motion_variant;

	//! Motion time - used in the Interpolated Position Mode.
	double estimated_time;

	int32_t motor_pos[NUM_OF_SERVOS];

	double joint_pos[NUM_OF_SERVOS];

	double goal_pos[6];

	//! Allowed time for the motion in seconds.
	//! If 0, then the motion time will be limited by the motor parameters.
	//! If > 0 and greater than a limit imposed by the motors, then the motion will be slowed down.
	//! In another case, the NACK will be replied.
	double duration;

	//! True if the contact is expected during the motion.
	//! The NACK will be replied if:
	//! - the contact was expected and did not happened
	//! - OR the contact was NOT expected and did happened.
	bool guarded_motion;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & variant;

		ar & get_pose_specification;
		switch (variant)
		{
			case POSE:
				ar & set_pose_specification;
				switch (set_pose_specification)
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
	lib::Homog_matrix current_pose;

	epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];

	bool contact;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & current_pose;
		ar & epos_controller;
		ar & contact;
	}
};

} // namespace spkm
} // namespace lib
} // namespace mrrocpp

#endif
