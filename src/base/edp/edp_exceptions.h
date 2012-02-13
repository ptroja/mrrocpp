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
 * Violated limit type.
 * @ingroup exception
 */
typedef enum _LIMIT_TYPE
{
	UPPER_LIMIT, LOWER_LIMIT
} LIMIT_TYPE;

//! Type of violated limit.
typedef boost::error_info <struct limit_type_, mrrocpp::edp::exception::LIMIT_TYPE> limit_type;

//! Convert limit type diagnostic field to human-readable string.
inline std::string to_string(limit_type const & e)
{
	switch (e.value())
	{
		case mrrocpp::edp::exception::UPPER_LIMIT:
			return "Upper";
		case mrrocpp::edp::exception::LOWER_LIMIT:
			return "Lower";
		default:
			return "Unknown";
	}
}

//! Number of motor that caused the exception.
typedef boost::error_info <struct motor_number_, int> motor_number;

//! Number of joint that caused the exception.
typedef boost::error_info <struct joint_number_, int> joint_number;

//! Desired value that caused the exception.
typedef boost::error_info <struct desired_value_, double> desired_value;

//! Value of the limit that couldn't be exceeded and caused the exception.
typedef boost::error_info <struct limit_value_, double> limit_value;

/*!
 * \brief Exception thrown in case of motor limits violation.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_motor_limit, "Motor limit exceeded")

/*!
 * \brief Exception thrown in case of joint limits violation.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_joint_limit, "Joint limit exceeded")

/*!
 * \brief Exception thrown when in an unsychronized robot state a command requiring synchronization is received.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_robot_unsynchronized, "Robot unsynchronized")

/*!
 * \brief Exception thrown when in an synchronization finished unsuccessfully.
 * \author tkornuta
 */
REGISTER_FATAL_ERROR(fe_synchronization_unsuccessful, "Robot synchronization failed")

/*!
 * \brief Exception thrown when robot is in fault state.
 * \author tkornuta
 */
REGISTER_FATAL_ERROR(fe_robot_in_fault_state, "Robot in fault state")

/*!
 * \brief Exception thrown in case of invalid pose specification.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_pose_specification, "Invalid pose specification")

/*!
 * \brief Exception thrown when an invalid command is retrieved.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_command, "Invalid command")

/*!
 * \brief Exception thrown in case of invalid motion type.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_motion_type, "Invalid motion type")

/*!
 * \brief (GOF) Good old-fashioned mrroc++ non fatal error 2.
 * \author yoyek
 */
REGISTER_NON_FATAL_ERROR(nfe_2, "Non fatal error - type 2")

/*!
 * \brief (GOF) Good old-fashioned mrroc++ fatal error.
 * \author yoyek
 */
REGISTER_FATAL_ERROR(fe, "Fatal error")

/*!
 * \brief (GOF) Good old-fashioned mrroc++  System error.
 * \author yoyek
 */
REGISTER_SYSTEM_ERROR(se, "System error")

/*!
 * Macro for handling MRROC++ system errors in EDP.
 *
 * \param ERROR Exception derived from the system_error classes.
 *
 * \author tkornuta
 * \date 27.10.2011
 */
#define HANDLE_EDP_SYSTEM_ERROR(ERROR) \
	std::cout<< ERROR.what() << std::endl; \
	msg->message(ERROR); \
	BOOST_THROW_EXCEPTION(se() << mrrocpp_error0(EDP_ERROR) << mrrocpp_error1(EDP_UNIDENTIFIED_ERROR)); \

/*!
 * Macro for handling MRROC++ fatal errors in EDP.
 *
 * \param ERROR Exception derived from the fatal_error classes.
 *
 * \author tkornuta
 * \date 27.10.2011
 */
#define HANDLE_EDP_FATAL_ERROR(ERROR) \
	std::cout<< ERROR.what() << std::endl; \
	msg->message(ERROR); \
	BOOST_THROW_EXCEPTION(fe() << mrrocpp_error1(EDP_UNIDENTIFIED_ERROR)); \

/*!
 * Macro for handling MRROC++ non-fatal errors in EDP.
 *
 * \param ERROR Exception derived from the non_fatal_error classes.
 *
 * \author tkornuta
 * \date 27.10.2011
 */
#define HANDLE_EDP_NON_FATAL_ERROR(ERROR) \
	std::cout<< ERROR.what() << std::endl; \
	msg->message(ERROR); \
	BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error1(EDP_UNIDENTIFIED_ERROR)); \

/*!
 * Macro for handling unknown errors in EDP.
 *
 * \author tkornuta
 * \date 02.12.2011
 */
#define HANDLE_EDP_UNKNOWN_ERROR() \
	msg->message(mrrocpp::lib::FATAL_ERROR, "Unknown error"); \
	BOOST_THROW_EXCEPTION(fe() << mrrocpp_error1(EDP_UNIDENTIFIED_ERROR)); \

} // namespace exception
} // namespace edp
} // namespace mrrocpp

#endif /* EDP_EXCEPTION_H_ */
