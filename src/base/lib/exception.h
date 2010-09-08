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

namespace mrrocpp {
namespace lib {
namespace exception {

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
