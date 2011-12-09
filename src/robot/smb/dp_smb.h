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
 * @brief SwarmItFix Mobile Base mp to ecp command
 * @ingroup smb
 */
/*!
 * @brief SwarmItFix Mobile Base action
 * @ingroup smb
 */
class action
{
public:
	//! Constructor with reasonable defaults
	action() :
		rotationPin(0),
		dThetaInd(0),
		dPkmTheta(0),
		duration(0)
	{
	}

	//! Get motion duration parameter
    double getDuration() const
    {
    	return duration;
    }

    //! Get PKM rotation
    double getdPkmTheta() const
    {
    	return dPkmTheta;
    }

    //! Get rotation pin
    unsigned int getRotationPin() const
    {
    	return rotationPin;
    }

    //! Get mobile base transrotation
    int getdThetaInd() const
    {
    	return dThetaInd;
    }

    //! Set motion duration parameter
    void setDuration(double duration)
    {
    	if(duration < 0) {
    		BOOST_THROW_EXCEPTION(action_parameter_error());
    	}

    	this->duration = duration;
    }

    //! Set PKM relative rotation
    void setdPkmTheta(double dPkmTheta)
    {
    	if (dPkmTheta < -2*M_PI || dPkmTheta > 2*M_PI) {
    		BOOST_THROW_EXCEPTION(action_parameter_error());
    	}

    	this->dPkmTheta = dPkmTheta;
    }

    //! Set PIN to rotate about
    void setRotationPin(unsigned int rotationPin)
    {
    	if(rotationPin < 0 || rotationPin > 3) {
    		BOOST_THROW_EXCEPTION(action_parameter_error());
    	}

    	this->rotationPin = rotationPin;
    }

    //! Set mobile base relative rotation
    void setdThetaInd(int dThetaInd)
    {
    	if (dThetaInd < -5 || dThetaInd > +5) {
    		BOOST_THROW_EXCEPTION(action_parameter_error());
    	}

    	this->dThetaInd = dThetaInd;
    }

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

	int32_t motor_pos[NUM_OF_SERVOS];

	double joint_pos[NUM_OF_SERVOS];

	// external
	int base_vs_bench_rotation;
	double pkm_vs_base_rotation;

	//! Allowed time for the motion in seconds.
	//! If 0, then the motion time will be limited by the motor parameters.
	//! If > 0 and greater than a limit imposed by the motors, then the motion will be slowed down.
	//! In another case, the NACK will be replied.
	double duration;

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

}__attribute__((__packed__));

/*!
 * @brief SwarmItFix Mobile Base single leg status
 * @ingroup smb
 */
struct leg_reply
{
	bool is_in;
	bool is_out;
	bool is_attached;

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
