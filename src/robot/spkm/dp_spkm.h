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
#include "../../base/lib/com_buf.h"

namespace mrrocpp {
namespace lib {
namespace spkm {

/*!
 * @brief SwarmItFix Epos simple external command data port
 * @ingroup spkm
 */
const std::string EPOS_EXTERNAL_COMMAND_DATA_PORT = "EPOS_EXTERNAL_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos status data request port
 * @ingroup spkm
 */
const std::string EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT = "EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Parallel Kinematic Machine mp to ecp variant
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

	//! Constructor with reasonable defaults
	_segment(const lib::Homog_matrix & _goal = lib::Homog_matrix());

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

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

/**
 * ECP command variant
 */
typedef enum _command_variant
{
	GOAL_POSE, STOP
} command_variant;

/*!
 *  Command for ECP agent
 */
typedef struct _next_state_t
{
	//! Command variant
	command_variant variant;

	//! Motion segment for SPKM
	spkm::segment_t segment;

	//! Constructor with safe defaults
	_next_state_t(command_variant _variant = STOP) :
			variant(_variant)
	{
	}

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & variant;
		switch (variant)
		{
			case GOAL_POSE:
				ar & segment;
				break;
			default:
				break;
		}
	}
} next_state_t;

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP variant buffer variant enum
 * @ingroup spkm
 */
enum CBUFFER_VARIANT
{
	POSE, QUICKSTOP, CLEAR_FAULT, BRAKE, DISABLE_BRAKE
};

/*!
 * Pose specification variants
 * @ingroup spkm
 */
typedef enum _POSE_SPECIFICATION
{
	WRIST_XYZ_EULER_ZYZ, TOOL_XYZ_EULER_ZYZ, JOINT, MOTOR
} POSE_SPECIFICATION;

/*!
 * @brief SwarmItFix Epos external mode controllers status
 * @ingroup spkm
 */
struct spkm_ext_epos_reply
{
//	POSE_SPECIFICATION pose_specification;

	//! SPKM current pose - in XYZ Euler ZYZ form.
	double current_pose[6];

	lib::epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];

	bool contact;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		//ar & pose_specification;
		ar & current_pose;
		ar & epos_controller;
		ar & contact;
	}
};

/*!
 * @brief SwarmItFix Epos external command
 * @ingroup spkm
 */
struct spkm_epos_simple_command
{
	lib::epos::EPOS_MOTION_VARIANT motion_variant;
	POSE_SPECIFICATION pose_specification;
	double desired_position[NUM_OF_SERVOS];

	double estimated_time;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & pose_specification;
		ar & motion_variant;
		ar & desired_position;
		ar & estimated_time;
	}
};

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP variant buffer
 * @ingroup spkm
 */
struct cbuffer
{
	//! Variant of the variant
	CBUFFER_VARIANT variant;

	//! Pose specification type
	POSE_SPECIFICATION set_pose_specification;

	//! Pose specification type
	POSE_SPECIFICATION get_pose_specification;

	//! Motion interpolation variant
	lib::epos::EPOS_MOTION_VARIANT motion_variant;

	//! Motion time - used in the Interpolated Position Mode.
	double estimated_time;

	//! SPKM desired motor positions.
	int32_t motor_pos[NUM_OF_SERVOS];

	//! SPKM desired joint positions.
	double joint_pos[NUM_OF_SERVOS];

	//! SPKM desired cartesian pose - in XYZ Euler ZYZ form.
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

private:
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
					case TOOL_XYZ_EULER_ZYZ:
					case WRIST_XYZ_EULER_ZYZ:
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
};

/*!
 * @brief SwarmItFix Head EDP command buffer
 * @ingroup spkm
 */
struct c_buffer : lib::c_buffer
{
	cbuffer spkm;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <lib::c_buffer>(*this);
		ar & spkm;
	}

};

/*!
 * @brief SwarmItFix Parallel Kinematic Machine EDP reply buffer
 * @ingroup spkm
 */
struct rbuffer
{
	//! SPKM current pose - in XYZ Euler ZYZ form.
	double current_pose[6];

	epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];

	bool contact;

private:
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

struct r_buffer : lib::r_buffer
{
	rbuffer spkm;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		// serialize base class informationZ
		ar & boost::serialization::base_object <lib::r_buffer>(*this);
		ar & spkm;
	}

};

} // namespace spkm
} // namespace lib
} // namespace mrrocpp

#endif
