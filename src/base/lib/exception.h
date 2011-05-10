/*!
 * @file exception.h
 * @brief Exception declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 *
 * @ingroup LIB
 */

#ifndef __TRANSFORMER_ERROR_H
#define __TRANSFORMER_ERROR_H

#include <stdint.h>
#include <boost/exception/all.hpp>
#include <boost/exception/diagnostic_information.hpp>
//#include <exception>

namespace mrrocpp {
namespace lib {
namespace exception {

//! Description used for diagnostic information in case of system errors.
const std::string SYSTEM_ERROR = "SYSTEM ERROR";

//! Description used for diagnostic information in case of fatal errors.
const std::string FATAL_ERROR = "FATAL ERROR";

//! Description used for diagnostic information in case of non fatal errors.
const std::string NON_FATAL_ERROR = "NON FATAL ERROR";

//! MRROC++ error class - by default three types are denoted (SYSTEM, FATAL, NON-FATAL).
typedef boost::error_info <struct mrrocpp_error_class_, char const *> mrrocpp_error_class;

//! MRROC++ error code - code identifying given error.
typedef boost::error_info <struct mrrocpp_error_code_, uint64_t> mrrocpp_error_code;

//! Description of the MRROC++ error - it will be sent (by default) to the SR.
typedef boost::error_info <struct mrrocpp_error_description_, char const *> mrrocpp_error_description;

/*!
 * EDP non fatal errors.
 */
typedef enum EDP_NON_FATAL_ERROR_CODE
{
	EDP_NFE_MOTOR_LIMIT,
	EDP_NFE_JOINT_LIMIT,
	EDP_NFE_POSE_SPECIFICATION,
	EDP_NFE_MOTION_TYPE,
	EDP_NFE_CURRENT_CARTESIAN_POSE_UNKNOWN,
	EDP_NFE_ROBOT_UNSYNCHRONIZED
} edp_nfe_error_code_t;

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
 * Macro for registration of MRROC++ system errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param ERROR_CODE Identifier of given error (of type uint64_t).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 */
#define REGISTER_SYSTEM_ERROR(CLASS_NAME, ERROR_CODE, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::mrrocpp_system_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_code(ERROR_CODE) << mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};


/*!
 * Macro for registration of MRROC++ fatal errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param ERROR_CODE Identifier of given error (of type uint64_t).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 */
#define REGISTER_FATAL_ERROR(CLASS_NAME, ERROR_CODE, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::mrrocpp_fatal_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_code(ERROR_CODE) <<mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};

/*!
 * Macro for registration of MRROC++ non fatal errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param ERROR_CODE Identifier of given error (of type uint64_t).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 */
#define REGISTER_NON_FATAL_ERROR(CLASS_NAME, ERROR_CODE, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::mrrocpp_non_fatal_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_code(ERROR_CODE) <<mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};


/*!
 * Macro for handling MRROC++ non fatal errors.
 *
 * \param ERROR Exception derived from the mrrocpp_*_error classes.
 *
 * \author tkornuta
 */
#define HANDLE_NON_FATAL_ERROR(ERROR) \
	std::cout << boost::current_exception_diagnostic_information() << std::endl; \
	msg->message(ERROR);

/*!
 * \brief Base class for all system errors.
 * \author tkornuta
 */
struct mrrocpp_system_error : virtual public std::exception, virtual public boost::exception
{
	mrrocpp_system_error()
	{
		*this << mrrocpp_error_class(SYSTEM_ERROR.c_str());
	}

	virtual const char* what() const throw ()
	{
		return diagnostic_information_what(*this);
	}

	~mrrocpp_system_error() throw ()
	{
	}
};

/*!
 * \brief Base class for all fatal errors.
 * \author tkornuta
 */
struct mrrocpp_fatal_error : virtual public std::exception, virtual public boost::exception
{
	mrrocpp_fatal_error()
	{
		*this << mrrocpp_error_class(FATAL_ERROR.c_str());
	}

	virtual const char* what() const throw ()
	{
		return diagnostic_information_what(*this);
	}

	~mrrocpp_fatal_error() throw ()
	{
	}
};

/*!
 * \brief Base class for all non fatal errors.
 * \author tkornuta
 */
struct mrrocpp_non_fatal_error : virtual public std::exception, virtual public boost::exception
{
	mrrocpp_non_fatal_error()
	{
		*this << mrrocpp_error_class(NON_FATAL_ERROR.c_str());
	}

	virtual const char* what() const throw ()
	{
		return diagnostic_information_what(*this);
	}

	~mrrocpp_non_fatal_error() throw ()
	{
	}
};



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
	Fatal_error(uint64_t err_no_0, uint64_t err_no_1);
};

/**
 * Non-fatal errors (type 1)
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 */
class NonFatal_error_1
{
public:
	//! Error in coordinate calculations
	const uint64_t error;

	/**
	 * Constructor
	 * @param err_no error value
	 */
	NonFatal_error_1(uint64_t err_no);
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
	NonFatal_error_2(uint64_t err_no);
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
	NonFatal_error_3(uint64_t err_no);
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
	NonFatal_error_4(uint64_t err_no);
};

} // namespace exception
} // namespace common
} // namespace mrrocpp

#endif
