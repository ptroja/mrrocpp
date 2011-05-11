/*!
 * @file edp_exceptions.h
 * @brief File containing consts, types and classes related to exceptions specific to SPKM.
 *
 * @author tkornuta
 * @date 10-05-2011
 *
 * @ingroup edp
 */

#ifndef EDP_EXCEPTION_H_
#define EDP_EXCEPTION_H_

#include "base/lib/exception.h"

namespace mrrocpp {
namespace edp {
namespace exception {

/*!
 * EDP non fatal errors - utilized in boost::exception-based error handling.
 * \author tkornuta
 */
typedef enum NON_FATAL_ERROR_CODE
{
	NFE_MOTOR_LIMIT,
	NFE_JOINT_LIMIT,
	NFE_ROBOT_UNSYNCHRONIZED,
	NFE_POSE_SPECIFICATION,
	NFE_MOTION_TYPE,
	NFE_CURRENT_CARTESIAN_POSE_UNKNOWN
} NFE_error_code_t;


//! Type of violated limit - upper.
const std::string UPPER_LIMIT = "Upper";

//! Type of violated limit - lower.
const std::string LOWER_LIMIT = "Lower";

//! Type of violated limit.
typedef boost::error_info <struct limit_type_, std::string> limit_type;

//! Number of motor that caused the exception.
typedef boost::error_info <struct motor_number_, int> motor_number;

//! Number of joint that caused the exception.
typedef boost::error_info <struct joint_number_, int> joint_number;

/*!
 * \brief Exception thrown in case of motor limits violation.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_motor_limit, mrrocpp::edp::exception::NFE_MOTOR_LIMIT, "Motor limit exceeded")

/*!
 * \brief Exception thrown in case of joint limits violation.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_joint_limit, mrrocpp::edp::exception::NFE_JOINT_LIMIT, "Joint limit exceeded")

/*!
 * \brief Exception thrown when in an unsychronized robot state a command requiring synchronization is received.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_robot_unsynchronized, mrrocpp::edp::exception::NFE_ROBOT_UNSYNCHRONIZED, "Robot unsynchronized")

/*!
 * \brief Exception thrown in case of invalid pose specification.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_pose_specification, mrrocpp::edp::exception::NFE_POSE_SPECIFICATION, "Invalid pose specification")

/*!
 * \brief Exception thrown in case of invalid motion type.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_motion_type, mrrocpp::edp::exception::NFE_MOTION_TYPE, "Invalid motion type")

} // namespace exception
} // namespace edp
} // namespace mrrocpp

#endif /* EDP_EXCEPTION_H_ */
