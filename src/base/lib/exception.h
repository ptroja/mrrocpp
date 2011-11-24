/*!
 * @file exception.h
 * @brief Exception declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 * @author Tomasz Winiarski <tomrobotics@gmail.com>
 * @author tkornuta <tkornuta@ia.pw.edu.com>
 *
 * @ingroup LIB
 */

#ifndef __TRANSFORMER_ERROR_H
#define __TRANSFORMER_ERROR_H

#include <stdint.h>
#include <sys/time.h>
#include <boost/exception/all.hpp>
#include <boost/exception/diagnostic_information.hpp>

namespace mrrocpp {
namespace lib {

// TODO: Move to the mrrocpp::lib::exception namespace.
/*!
 * Classes of errors in the MRROC++ framework.
 */
typedef enum _ERROR_CLASS_T
{
	NEW_MESSAGE, SYSTEM_ERROR, FATAL_ERROR, NON_FATAL_ERROR
} error_class_t;

}
}

namespace mrrocpp {
namespace lib {
namespace exception {

//! A single line description of error.
typedef boost::error_info <struct mrrocpp_error_description_, char const *> mrrocpp_error_description;

//! Moment in which error was detected.
typedef boost::error_info <struct time_, struct timeval> mrrocpp_error_time;

//! error0 for old mrroc++ exceptions
typedef boost::error_info <struct error0_, uint64_t> mrrocpp_error0;

//! error1 for old mrroc++ exceptions
typedef boost::error_info <struct error1_, uint64_t> mrrocpp_error1;

/*!
 * \brief Base class for all system exceptions/errors.
 * \author tkornuta
 * \date 12.05.2011
 */
template <error_class_t ercl>
class mrrocpp_error : virtual public std::exception, virtual public boost::exception
{
public:
	/*!
	 * Class of the error.
	 */
	const error_class_t error_class;

	/*!
	 * Constructor.
	 */
	mrrocpp_error() :
			error_class(ercl)
	{
		// Get current time.
		struct timeval tv;
		if (gettimeofday(&tv, NULL) == -1) {
			perror("gettimeofday()");
		}
		// Add it to diagnostic information.
		*this << mrrocpp_error_time(tv);
	}

	/*!
	 * Destructor.
	 */
	~mrrocpp_error() throw ()
	{
	}

	/*!
	 * Returns diagnostic information.
	 */
	virtual const char* what() const throw ()
	{
		return diagnostic_information_what(*this);
	}

	// TODO: timestampe based on boost::posix_time

};

/*!
 * \brief Base class for all system errors.
 * \author tkornuta
 * \date 12.05.2011
 */
typedef mrrocpp_error <SYSTEM_ERROR> mrrocpp_system_error;
/*!
 * \brief Base class for all fatal errors.
 * \author tkornuta
 * \date 12.05.2011
 */
typedef mrrocpp_error <FATAL_ERROR> mrrocpp_fatal_error;

/*!
 * \brief Base class for all non fatal errors.
 * \author tkornuta
 * \date 12.05.2011
 */
typedef mrrocpp_error <NON_FATAL_ERROR> mrrocpp_non_fatal_error;

/*!
 * Macro for registration of MRROC++ system errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 * \date 12.05.2011
 */
#define REGISTER_SYSTEM_ERROR(CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::mrrocpp_system_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};

/*!
 * Macro for registration of MRROC++ fatal errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 * \date 12.05.2011
 */
#define REGISTER_FATAL_ERROR(CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::mrrocpp_fatal_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};

/*!
 * Macro for registration of MRROC++ non fatal errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param DESCRIPTION Description added to the mrrocpp_error_description error info field.
 *
 * \author tkornuta
 * \date 12.05.2011
 */
#define REGISTER_NON_FATAL_ERROR(CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::mrrocpp_non_fatal_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::mrrocpp_error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};

/*!
 * Macro for handling MRROC++ system errors.
 *
 * \param ERROR Exception derived from the mrrocpp_system_error classes.
 *
 * \author tkornuta
 * \date 27.10.2011
 */

#define HANDLE_MRROCPP_SYSTEM_ERROR(ERROR) \
	std::cout<< ERROR.what() << std::endl; \
	msg->message(ERROR);

/*!
 * Macro for handling MRROC++ fatal errors.
 *
 * \param ERROR Exception derived from the mrrocpp_fatal_error classes.
 *
 * \author tkornuta
 * \date 27.10.2011
 */

#define HANDLE_MRROCPP_FATAL_ERROR(ERROR) \
	std::cout<< ERROR.what() << std::endl; \
	msg->message(ERROR);

/*!
 * Macro for handling MRROC++ non-fatal errors.
 *
 * \param ERROR Exception derived from the mrrocpp_non_fatal_error classes.
 *
 * \author tkornuta
 * \date 27.10.2011
 */

#define HANDLE_MRROCPP_NON_FATAL_ERROR(ERROR) \
	std::cout<< ERROR.what() << std::endl; \
	msg->message(ERROR);


} // namespace exception
} // namespace common
} // namespace mrrocpp

#endif
