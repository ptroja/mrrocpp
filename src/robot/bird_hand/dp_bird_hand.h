#if !defined(__BIRD_HAND_DATA_PORT_H)
#define __BIRD_HAND_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for Bird Hand three finger gripper
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup bird_hand
 *
 */

#include <boost/serialization/serialization.hpp>
#include "const_bird_hand.h"
#include "../../base/lib/com_buf.h"

namespace mrrocpp {
namespace lib {
namespace bird_hand {

/*!
 * @brief Bird Hand motion command data port
 * @ingroup bird_hand
 */
const std::string COMMAND_DATA_PORT = "bird_hand_command_data_port";
/*!
 * @brief Bird Hand motion status data request port
 * @ingroup bird_hand
 */
const std::string STATUS_DATA_REQUEST_PORT = "bird_hand_status_data_request_port";

/*!
 * @brief Bird Hand configuration command data port
 * @ingroup bird_hand
 */
const std::string CONFIGURATION_DATA_PORT = "bird_hand_configuration_data_port";
/*!
 * @brief Bird Hand configuration status data request port
 * @ingroup bird_hand
 */
const std::string CONFIGURATION_DATA_REQUEST_PORT = "bird_hand_configuration_data_request_port";

/*!
 * @brief Bird Hand three commanded motion variants enumeration
 * @ingroup bird_hand
 */
enum MOTION_VARIANT
{
	SIGLE_STEP_POSTION_INCREMENT = 0, MACROSTEP_POSITION_INCREMENT = 1, MACROSTEP_ABSOLUTE_POSITION = 2
};

/*!
 * @brief Bird Hand poition/torque command for single joint
 * @ingroup bird_hand
 */
struct single_joint_command
{
	MOTION_VARIANT profile_type;
	double reciprocal_of_damping;
	double desired_torque;
	double desired_position;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & profile_type;
		ar & reciprocal_of_damping;
		ar & desired_torque;
		ar & desired_position;
	}

};

/*!
 * @brief Bird Hand single joint reply status
 * @ingroup bird_hand
 */
struct single_joint_status
{
	double meassured_position;
	double meassured_torque;
	double measured_current;
	bool upper_limit_of_absolute_value_of_desired_position_increment;
	bool upper_limit_of_absolute_value_of_computed_position_increment;
	bool upper_limit_of_absolute_position;
	bool lower_limit_of_absolute_position;
	bool upper_limit_of_absolute_value_of_desired_torque;
	bool lower_limit_of_absolute_value_of_desired_torque;
	bool upper_limit_of_absolute_value_of_meassured_torque;
	bool upper_limit_of_measured_current;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & meassured_position;
		ar & meassured_torque;
		ar & measured_current;
		ar & upper_limit_of_absolute_value_of_desired_position_increment;
		ar & upper_limit_of_absolute_value_of_computed_position_increment;
		ar & upper_limit_of_absolute_position;
		ar & lower_limit_of_absolute_position;
		ar & upper_limit_of_absolute_value_of_desired_torque;
		ar & lower_limit_of_absolute_value_of_desired_torque;
		ar & upper_limit_of_absolute_value_of_meassured_torque;
		ar & upper_limit_of_measured_current;
	}

};

/*!
 * @brief Bird Hand configuration command for single joint
 * @ingroup bird_hand
 */
struct single_joint_configuration
{
	int32_t p_factor;
	int32_t i_factor;
	int32_t d_factor;
	int32_t value_of_upper_limit_of_absolute_position;
	int32_t value_of_lower_limit_of_absolute_position;
	int32_t value_of_upper_limit_of_measured_current;
	int32_t value_of_upper_limit_of_absolute_value_of_torque;
	int32_t value_of_lower_limit_of_absolute_value_of_torque;
	int32_t value_of_lower_limit_of_absolute_value_of_meassured_torque;
	int32_t value_of_upper_limit_of_position_increment;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & p_factor;
		ar & i_factor;
		ar & d_factor;
		ar & value_of_upper_limit_of_absolute_position;
		ar & value_of_lower_limit_of_absolute_position;
		ar & value_of_upper_limit_of_measured_current;
		ar & value_of_upper_limit_of_absolute_value_of_torque;
		ar & value_of_lower_limit_of_absolute_value_of_torque;
		ar & value_of_lower_limit_of_absolute_value_of_meassured_torque;
		ar & value_of_upper_limit_of_position_increment;
	}

};

/*!
 * @brief multi joint position/torque command for whole gripper
 * @ingroup bird_hand
 */
struct command
{
	int motion_steps;
	int ecp_query_step;
	single_joint_command thumb_f[THUMB_F_NUM_OF_SERVOS];
	single_joint_command index_f[INDEX_F_NUM_OF_SERVOS];
	single_joint_command ring_f[RING_F_NUM_OF_SERVOS];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & motion_steps;
		ar & ecp_query_step;
		ar & thumb_f;
		ar & index_f;
		ar & ring_f;
	}

};

/*!
 * @brief multi joint reply status for whole gripper
 * @ingroup bird_hand
 */
struct status
{
	single_joint_status thumb_f[THUMB_F_NUM_OF_SERVOS];
	single_joint_status index_f[INDEX_F_NUM_OF_SERVOS];
	single_joint_status ring_f[RING_F_NUM_OF_SERVOS];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & thumb_f;
		ar & index_f;
		ar & ring_f;
	}

};

/*!
 * @brief multi joint configuration command for whole gripper
 * @ingroup bird_hand
 */
struct configuration
{
	single_joint_configuration thumb_f[THUMB_F_NUM_OF_SERVOS];
	single_joint_configuration index_f[INDEX_F_NUM_OF_SERVOS];
	single_joint_configuration ring_f[RING_F_NUM_OF_SERVOS];

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & thumb_f;
		ar & index_f;
		ar & ring_f;
	}
};

typedef struct
{
	/*!
	 * @brief Motion duration in EDP steps
	 */
	int32_t motion_steps;
	/*!
	 * @brief EDP query step number
	 */
	int32_t ecp_query_step;
	single_joint_command finger[NUM_OF_SERVOS];

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & motion_steps;
		ar & ecp_query_step;
		ar & finger;
	}

} command_structure_t;

/*!
 * @brief Bird Hand EDP command buffer
 * @ingroup bird_hand
 */
struct cbuffer
{
	/*!
	 * @brief Motion command
	 */
	command_structure_t command_structure;

	/*!
	 * @brief Configuration command
	 */
	struct
	{
		single_joint_configuration finger[NUM_OF_SERVOS];
	} configuration_command_structure;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & command_structure;
		ar & configuration_command_structure.finger;
	}

};

struct c_buffer : lib::c_buffer
{
	cbuffer bird_hand;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::base_object <lib::c_buffer>(*this);
		ar & bird_hand;
	}

};

/*!
 * @brief Bird Hand EDP reply buffer
 * @ingroup bird_hand
 */
struct rbuffer
{
	struct
	{
		single_joint_status finger[NUM_OF_SERVOS];
	} status_reply_structure;
	struct
	{
		single_joint_configuration finger[NUM_OF_SERVOS];
	} configuration_reply_structure;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & status_reply_structure.finger;

		ar & configuration_reply_structure.finger;
	}

};

struct r_buffer : lib::r_buffer
{
	rbuffer bird_hand;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		// serialize base class informationZ
		ar & boost::serialization::base_object <lib::r_buffer>(*this);
		ar & bird_hand;
	}

};

} // namespace bird_hand
} // namespace lib
} // namespace mrrocpp

#endif
