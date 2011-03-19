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

namespace mrrocpp {
namespace kinematics {
namespace spkm {

//! Type of violated limit - upper.
const std::string UPPER_LIMIT = "UPPER";

//! Type of violated limit - lower.
const std::string LOWER_LIMIT = "LOWER";


//! Type of violated limit.
typedef boost::error_info <struct limit, std::string> spkm_limit_type;

//! Number of motor that caused the exception.
typedef boost::error_info <struct motor_no, int> spkm_motor_number;

//! Number of joint that caused the exception.
typedef boost::error_info <struct joint_no, int> spkm_joint_number;

//! Desired values that caused the exception.
typedef boost::error_info <struct desired_value, double> spkm_desired_value;


/*!
 * \brief Exception thrown in case of motor limits violation.
 * \author tkornuta
 */
struct spkm_motor_limit_error : mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~spkm_motor_limit_error() throw () { }
};

/*!
 * \brief Exception thrown in case of joint limits violation.
 * \author tkornuta
 */
struct spkm_joint_limit_error : mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~spkm_joint_limit_error() throw () { }
};

/*!
 * \brief Exception thrown in case of invalid pose specification.
 * \author tkornuta
 */
struct spkm_pose_specification_error : mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~spkm_pose_specification_error() throw () { }
};

/*!
 * \brief Exception thrown in case of invalid motion type.
 * \author tkornuta
 */
struct spkm_motion_type_error : mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~spkm_motion_type_error() throw () { }
};

/*!
 * \brief Exception thrown when in an unsychronized robot state a command requiring synchronization is received.
 * \author tkornuta
 */
struct spkm_unsynchronized_error : mrrocpp::lib::exception::mrrocpp_non_fatal_error
{
	~spkm_unsynchronized_error() throw () { }
};



} // namespace spkm
} // namespace kinematic
} // namespace mrrocpp


#endif /* SPKM_EXCEPTION_H_ */
