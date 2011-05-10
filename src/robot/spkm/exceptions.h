/*!
 * @file
 * @brief File containing consts, types and classes related to SPKM exceptions.
 *
 * @author tkornuta
 * @date 08-02-2011
 *
 * @ingroup spkm
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
namespace edp {
namespace spkm {

//! Desired value that caused the exception.
typedef boost::error_info <struct desired_value_, double> desired_value;

//! Pose specification type.
typedef boost::error_info <struct pose_specification_, mrrocpp::lib::spkm::POSE_SPECIFICATION> pose_specification;

/*!
 * \brief Exception thrown in case of motor limits violation.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_motor_limit, mrrocpp::lib::exception::EDP_NFE_MOTOR_LIMIT, "Motor limit exceeded")

/*!
 * \brief Exception thrown in case of joint limits violation.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_joint_limit, mrrocpp::lib::exception::EDP_NFE_JOINT_LIMIT, "Joint limit exceeded")

/*!
 * \brief Exception thrown in case of invalid pose specification.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_pose_specification, mrrocpp::lib::exception::EDP_NFE_POSE_SPECIFICATION, "Invalid pose specification")

/*!
 * \brief Exception thrown in case of invalid motion type.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_motion_type, mrrocpp::lib::exception::EDP_NFE_MOTION_TYPE, "Invalid motion type")

/*!
 * \brief Exception thrown when cartesian pose is required, but unknown.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_current_cartesian_pose_unknown, mrrocpp::lib::exception::EDP_NFE_CURRENT_CARTESIAN_POSE_UNKNOWN, "Required current cartesian pose is unknown")

/*!
 * \brief Exception thrown when in an unsychronized robot state a command requiring synchronization is received.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_robot_unsynchronized, mrrocpp::lib::exception::EDP_NFE_ROBOT_UNSYNCHRONIZED, "Robot unsynchronized")



} // namespace spkm
} // namespace edp
} // namespace mrrocpp


#endif /* SPKM_EXCEPTION_H_ */
