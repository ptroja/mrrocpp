#if !defined(__SHEAD_DATA_PORT_H)
#define __SHEAD_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Head
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup shead
 */

#include <string>

#include "robot/maxon/dp_epos.h"
#include "const_shead.h"

namespace mrrocpp {
namespace lib {
namespace shead {

/*!
 * Pose specification variants
 * @ingroup shead
 */
typedef enum _POSE_SPECIFICATION
{
	FRAME, JOINT, MOTOR
} POSE_SPECIFICATION;

/*!
 * @brief SwarmItFix Head head soldification command data port
 * @ingroup shead
 */
const std::string SOLIDIFICATION_ACTIVATION_DATA_PORT = "SHEAD_SOLIDIFICATION_ACTIVATION_DATA_PORT";

/*!
 * @brief SwarmItFix Head head vacuum activation command data port
 * @ingroup shead
 */
const std::string VACUUM_ACTIVATION_DATA_PORT = "SHEAD_VACUUM_ACTIVATION_DATA_PORT";

/*!
 * @brief SwarmItFix Head status data request port
 * @ingroup shead
 */
const std::string REPLY_DATA_REQUEST_PORT = "SHEAD_REPLY_DATA_REQUEST_PORT";

/*!
 * @brief SwarmItFix Head EDP state of the head soldification enum
 * @ingroup shead
 */
enum STATE_OF_THE_SOLDIFICATION
{
	SOLDIFICATION_STATE_ON, SOLDIFICATION_STATE_OFF, SOLDIFICATION_STATE_INTERMEDIATE
};

/*!
 * @brief SwarmItFix Head EDP state of the vacuum enum
 * @ingroup shead
 */
enum STATE_OF_THE_VACUUM
{
	VACUUM_STATE_ON, VACUUM_STATE_OFF, VACUUM_STATE_INTERMEDIATE
};

/*!
 * @brief SwarmItFix Head EDP head soldification command enum
 * @ingroup shead
 */
enum SOLIDIFICATION_ACTIVATION
{
	SOLIDIFICATION_ON, SOLIDIFICATION_OFF
};
// namespace mrrocpp

/*!
 * @brief SwarmItFix Head EDP vacuum activation command enum
 * @ingroup shead
 */
enum VACUUM_ACTIVATION
{
	VACUUM_ON, VACUUM_OFF
};
// namespace mrrocpp

/*!
 * @brief SwarmItFix Head reply buffer
 * @ingroup shead
 */
struct reply
{
	STATE_OF_THE_SOLDIFICATION soldification_state;
	STATE_OF_THE_VACUUM vacuum_state;
	bool contacts[3];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & soldification_state;
		ar & vacuum_state;
		ar & contacts;
	}

}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Head EDP command buffer variant enum
 * @ingroup shead
 */
enum CBUFFER_VARIANT
{
	POSE, QUICKSTOP, CLEAR_FAULT, SOLIDIFICATION, VACUUM
};

/*!
 * @brief SwarmItFix Head EDP command buffer
 * @ingroup shead
 */
struct cbuffer
{

	//! Variant of the command
	CBUFFER_VARIANT variant;

	lib::shead::SOLIDIFICATION_ACTIVATION head_solidification;
	lib::shead::VACUUM_ACTIVATION vacuum_activation;

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
		ar & head_solidification;
		ar & vacuum_activation;
		ar & get_pose_specification;
		switch (variant)
		{
			case POSE:
				ar & set_pose_specification;
				switch (set_pose_specification)
				{
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
 * @brief SwarmItFix Head EDP reply buffer
 * @ingroup shead
 */
struct rbuffer
{
	reply shead_reply;
	epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & shead_reply;
		ar & epos_controller;
	}

}__attribute__((__packed__));

} // namespace shead
}
}

#endif
