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
#include "const_smb.h"

namespace mrrocpp {
namespace lib {
namespace smb {

/*!
 * @brief SwarmItFix Mobile Base fest command data port
 * @ingroup smb
 */
const std::string FESTO_COMMAND_DATA_PORT = "FESTO_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Mobile Base status data request port
 * @ingroup smb
 */
const std::string MULTI_LEG_REPLY_DATA_REQUEST_PORT = "MULTI_LEG_REPLY_DATA_REQUEST_PORT";

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
	bool undetachable[LEG_CLAMP_NUMBER];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & leg;
		ar & undetachable;
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

/*!
 * @brief SwarmItFix Mobile Base EDP command buffer
 * @ingroup smb
 */
struct cbuffer
{
	//! Variant of the command
	CBUFFER_VARIANT variant;

	festo_command_td festo_command;

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
		ar & festo_command;
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
 * @brief SwarmItFix Mobile Base EDP reply buffer
 * @ingroup smb
 */
struct rbuffer
{
	multi_leg_reply_td multi_leg_reply;
	epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & multi_leg_reply;
		ar & epos_controller;
	}

};

} // namespace smb
}
}

#endif
