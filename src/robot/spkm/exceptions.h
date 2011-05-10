/*!
 * @file
 * @brief File containing consts, types and classes related to SPKM exceptions.
 *
 * @author tkornuta
 * @date 08-02-2011
 *
 * @ingroup SIF_KINEMATICS spkm
 */
/*
 * spkm_exception.h
 *
 *  Created on:
 *      Author: tkornuta
 */

#ifndef SPKM_EXCEPTION_H_
#define SPKM_EXCEPTION_H_

#include "base/lib/exception.h"
#include "robot/spkm/const_spkm.h"

namespace mrrocpp {
namespace kinematics {
namespace spkm {

//! Type of violated limit - upper.
const std::string UPPER_LIMIT = "UPPER";

//! Type of violated limit - lower.
const std::string LOWER_LIMIT = "LOWER";


//! Type of violated limit.
typedef boost::error_info <struct spkm_limit, std::string> limit_type;

//! Number of motor that caused the exception.
typedef boost::error_info <struct spkm_motor_no, int> motor_number;

//! Number of joint that caused the exception.
typedef boost::error_info <struct spkm_joint_no, int> joint_number;

//! Desired value that caused the exception.
typedef boost::error_info <struct spkm_desired_value, double> desired_value;

//! Motion type.
typedef boost::error_info <struct spkm_pose_specification, mrrocpp::lib::spkm::POSE_SPECIFICATION> pose_specification;

/*!
 * Macro for registration of MRROC++ errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param BASE_CLASS_NAME Name of the base class (without the namespace(s) scope).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 */
#define REGISTER_MRROCPP_ERROR(CLASS_NAME, BASE_CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::BASE_CLASS_NAME \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};


/*!
 * \brief Exception thrown in case of motor limits violation.
 * \author tkornuta
 */
REGISTER_MRROCPP_ERROR(motor_limit_error, mrrocpp_non_fatal_error, "Motor limit exceeded")

/*!
 * \brief Exception thrown in case of joint limits violation.
 * \author tkornuta
 */
REGISTER_MRROCPP_ERROR(joint_limit_error, mrrocpp_non_fatal_error, "Joint limit exceeded")

/*!
 * \brief Exception thrown in case of invalid pose specification.
 * \author tkornuta
 */
REGISTER_MRROCPP_ERROR(pose_specification_error, mrrocpp_non_fatal_error, "Invalid pose specification")

/*!
 * \brief Exception thrown in case of invalid motion type.
 * \author tkornuta
 */
REGISTER_MRROCPP_ERROR(motion_type_error, mrrocpp_non_fatal_error, "Invalid motion type")

/*!
 * \brief Exception thrown when cartesian pose is required, but unknown.
 * \author tkornuta
 */
REGISTER_MRROCPP_ERROR(current_cartesian_pose_unknown, mrrocpp_non_fatal_error, "Required current cartesian pose is unknown")

/*!
 * \brief Exception thrown when in an unsychronized robot state a command requiring synchronization is received.
 * \author tkornuta
 */
REGISTER_MRROCPP_ERROR(unsynchronized_error, mrrocpp_non_fatal_error, "PKM unsynchronized")



} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp


#endif /* SPKM_EXCEPTION_H_ */
