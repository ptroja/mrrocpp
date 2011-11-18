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

//! Desired value that caused the exception.
typedef boost::error_info <struct desired_value_, double> desired_value;

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
 * \brief Exception thrown in case of invalid pose specification.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_pose_specification, "Invalid pose specification")

/*!
 * \brief Exception thrown in case of invalid motion type.
 * \author tkornuta
 */
REGISTER_NON_FATAL_ERROR(nfe_invalid_motion_type, "Invalid motion type")

/*!
 * \brief old mrroc++ non fatal error 1
 * \author yoyek
 */
REGISTER_NON_FATAL_ERROR(nfe_1, "NonFatal_error_1")

/********************************** OLD MRROC++ ERRORS **********************************/

/**
 * System error (inter-process communication, filesystem, etc.)
 */
class System_error
{
};

/**
 * Fatal exception in framework or application
 */
class Fatal_error
{
public:
	//! Servo error number (1)
	const uint64_t error0;

	//! Servo error number (2)
	const uint64_t error1;

	/**
	 * Constructor
	 * @param err_no_0 servo error number (1)
	 * @param err_no_1 servo error number (2)
	 * @return
	 */
	Fatal_error(uint64_t err_no_0, uint64_t err_no_1) :
			error0(err_no_0), error1(err_no_1)
	{
	}
};

/**
 * Non-fatal errors (type 2)
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 */
class NonFatal_error_2
{
public:
	//! Error in coordinate calculations
	const uint64_t error;

	/**
	 * Constructor
	 * @param err_no error value
	 */
	NonFatal_error_2(uint64_t err_no) :
			error(err_no)
	{
	}
};

/**
 * Non-fatal errors (type 3)
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 */
class NonFatal_error_3
{
public:
	//! Error in coordinate calculations
	const uint64_t error;

	/**
	 * Constructor
	 * @param err_no error value
	 */
	NonFatal_error_3(uint64_t err_no) :
			error(err_no)
	{
	}
};

/**
 * Non-fatal errors (type 4)
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 */
class NonFatal_error_4
{
public:
	//! Error in coordinate calculations
	const uint64_t error;

	/**
	 * Constructor
	 * @param err_no error value
	 */
	NonFatal_error_4(uint64_t err_no) :
			error(err_no)
	{
	}
};

} // namespace exception
} // namespace edp
} // namespace mrrocpp

#endif /* EDP_EXCEPTION_H_ */
