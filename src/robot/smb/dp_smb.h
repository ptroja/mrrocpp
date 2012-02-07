#if !defined(__SMB_DATA_PORT_H)
#define __SMB_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Mobile Base
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup smb
 */

#include <cmath>

#include <boost/throw_exception.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include "robot/maxon/dp_epos.h"
#include "const_smb.h"
#include "base/lib/exception.h"
#include "base/lib/com_buf.h"

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

REGISTER_NON_FATAL_ERROR(action_parameter_error, "SMB action parameters error")

/*!
 * @brief SwarmItFix Epos simple external command data port
 * @ingroup smb
 */
const std::string EPOS_EXTERNAL_COMMAND_DATA_PORT = "EPOS_EXTERNAL_COMMAND_DATA_PORT";

/*!
 * @brief SwarmItFix Epos status data request port
 * @ingroup smb
 */
const std::string EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT = "EPOS_EXTERNAL_REPLY_DATA_REQUEST_PORT";



/*!
 * @brief SwarmItFix Mobile Base action
 * @ingroup smb
 */
class action
{
public:
	//! Constructor with reasonable defaults
	action();

	//! Get motion duration parameter
    double getDuration() const;

    //! Get PKM rotation
    double getdPkmTheta() const;

    //! Get rotation pin
    unsigned int getRotationPin() const;

    //! Get mobile base transrotation
    int getdThetaInd() const;

    //! Set motion duration parameter
    void setDuration(double duration);

    //! Set PKM relative rotation
    void setdPkmTheta(double dPkmTheta);

    //! Set PIN to rotate about
    void setRotationPin(unsigned int rotationPin);

    //! Set mobile base relative rotation
    void setdThetaInd(int dThetaInd);

private:
	//! Pin around which to rotate {0,1,2,3}
	unsigned int rotationPin;

	//! Rotation around pin {-5..+5}
	int dThetaInd;

	//! Rotation of PKM around mobile base
	double dPkmTheta;

	//! Allowed time for the motion in seconds.
	//! If 0, then the time will be limited by the motor limits.
	//! If > 0 and greater than a limit imposed by the motors, then the motion will be slowed down.
	//! In another case, the NACK will be replied.
	double duration;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(rotationPin);
		// Check if rotating around the pin
		if(rotationPin) {
			ar & BOOST_SERIALIZATION_NVP(dThetaInd);
		}
		ar & BOOST_SERIALIZATION_NVP(dPkmTheta);
		// Check if executing motion at all
		if((rotationPin && dThetaInd) || dPkmTheta) {
			ar & BOOST_SERIALIZATION_NVP(duration);
		}
	}
};

/**
 * ECP command variant
 */
typedef enum _command_variant { ACTION_LIST, STOP } command_variant;

/*!
 *  Command for ECP agent
 */
typedef struct _next_state_t
{
	//! Command variant
	command_variant variant;

	//! Type for sequence of actions ofmobile base
	typedef std::vector<action> action_sequence_t;

	//! Sequence of actions for mobile base
	action_sequence_t actions;

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
		switch (variant) {
			case ACTION_LIST:
				ar & actions;
				break;
			default:
				break;
		}
	}
} next_state_t;

/*!
 * @brief SwarmItFix Mobile Base single leg festo command
 * @ingroup smb
 */
enum FESTO_LEG
{
	IN, OUT
};

/*!
 * @brief SwarmItFix Mobile Base multi pin insertion command
 * @ingroup smb
 */
struct festo_command_td
{
	FESTO_LEG leg[LEG_CLAMP_NUMBER];
	bool undetachable[LEG_CLAMP_NUMBER];

	//! Initialize "safe" command
	festo_command_td();

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & leg;
		ar & undetachable;
	}

};

/*!
 * @brief SwarmItFix Epos all controllers status
 * @ingroup epos
 */
struct smb_ext_epos_reply
{
	lib::Homog_matrix current_frame;
	lib::epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & current_frame;
		ar & epos_controller;
	}
};

/*!
 * @brief SwarmItFix Epos motor and joint and external command, called from UI
 * @ingroup smb
 */
struct motor_command
{
	//! Rotation of the legs (in external values -6, -5, ..., 5, 6).
	int base_vs_bench_rotation;

	//! Desired rotation of the upper SMP by given angle [radians].
	double pkm_vs_base_rotation;

	//! FIXME: Estimated time of motion (UNUSED!).
	double estimated_time;

	//! Constructor to initialize "safe" command.
	motor_command();

private:
	//! Give access to boost::serialization framework.
	friend class boost::serialization::access;

	//! Serialization of the data structure.
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & base_vs_bench_rotation;
		ar & pkm_vs_base_rotation;
		ar & estimated_time;
	}
};

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

	//! Two-element vector containing desired motor positions.
	int32_t motor_pos[NUM_OF_SERVOS];

	//! Two-element vector containing desired joint positions.
	double joint_pos[NUM_OF_SERVOS];

	// Desired external legs rotation.
	int base_vs_bench_rotation;

	// Desired external PKM rotation.
	double pkm_vs_base_rotation;

	//! Allowed time for the motion in seconds.
	//! If 0, then the motion time will be limited by the motor parameters.
	//! If > 0 and greater than a limit imposed by the motors, then the motion will be slowed down.
	//! In another case, the NACK will be replied.
	double duration;

private:
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
					case EXTERNAL:
						ar & base_vs_bench_rotation;
						ar & pkm_vs_base_rotation;
						break;
					case JOINT:
						ar & joint_pos;
						break;
					case MOTOR:
						ar & motor_pos;
						break;
				}
				break;
			default:
				break;
		};
	}

};

struct c_buffer : lib::c_buffer
{
	cbuffer smb;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <lib::c_buffer>(*this);
		ar & smb;
	}

};

/*!
 * @brief SwarmItFix Mobile Base single leg status
 * @ingroup smb
 */
struct leg_reply
{
	bool is_in;
	bool is_out;
	bool is_attached;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & is_in;
		ar & is_out;
		ar & is_attached;
	}

};

/*!
 * @brief SwarmItFix Mobile Base multi leg reply
 * @ingroup smb
 */
struct multi_leg_reply_td
{
	leg_reply leg[LEG_CLAMP_NUMBER];

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & leg;
	}

};

/*!
 * @brief SwarmItFix Mobile Base EDP reply buffer
 * @ingroup smb
 */
struct rbuffer
{
	multi_leg_reply_td multi_leg_reply;
	epos::single_controller_epos_reply epos_controller[NUM_OF_SERVOS];

private:
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

struct r_buffer : lib::r_buffer
{
	rbuffer smb;

private:
	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		// serialize base class information
		ar & boost::serialization::base_object <lib::r_buffer>(*this);
		ar & smb;
	}

};

} // namespace smb
}
}

#endif
