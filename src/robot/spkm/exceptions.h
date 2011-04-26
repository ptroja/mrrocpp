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
 * \brief Exception thrown in case of motor limits violation.
 * \author tkornuta
 */
struct motor_limit_error : virtual mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~motor_limit_error() throw () { }
};

/*!
 * \brief Exception thrown in case of joint limits violation.
 * \author tkornuta
 */
struct joint_limit_error : virtual mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~joint_limit_error() throw () { }
};

/*!
 * \brief Exception thrown in case of invalid pose specification.
 * \author tkornuta
 */
struct pose_specification_error : virtual mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~pose_specification_error() throw () { }
};

/*!
 * \brief Exception thrown in case of invalid motion type.
 * \author tkornuta
 */
struct motion_type_error : mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~motion_type_error() throw () { }
};

/*!
 * \brief Exception thrown when cartesian pose is required, but unknown.
 * \author tkornuta
 */
struct current_cartesian_pose_unknown : mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~current_cartesian_pose_unknown() throw () { }
};

/*!
 * \brief Exception thrown when in an unsychronized robot state a command requiring synchronization is received.
 * \author tkornuta
 */
struct unsynchronized_error : mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~unsynchronized_error() throw () { }
};



} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp


#endif /* SPKM_EXCEPTION_H_ */
