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
#include <boost/exception/all.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>

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

namespace exception {

//! A single line description of error.
typedef boost::error_info <struct error_description_, char const *> error_description;

//! Time when error was detected.
typedef boost::error_info <struct timestamp, boost::system_time> error_time;

//! Convert exception's timestamp to human-readable string
inline std::string to_string(error_time const & e)
{
	return boost::posix_time::to_simple_string(e.value());
}

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
class error : virtual public std::exception, virtual public boost::exception
{
public:
	/*!
	 * Class of the error.
	 */
	const error_class_t error_class;

	/*!
	 * Constructor.
	 */
	error() :
			error_class(ercl)
	{
		// Add it to diagnostic information.
		*this << error_time(boost::get_system_time());
	}

	/*!
	 * Destructor.
	 */
	~error() throw ()
	{
	}

	/*!
	 * Returns diagnostic information.
	 */
	virtual const char* what() const throw ()
	{
		return diagnostic_information_what(*this);
	}
};

/*!
 * \brief Base class for all system errors.
 * \author tkornuta
 * \date 12.05.2011
 */
typedef error <SYSTEM_ERROR> system_error;
/*!
 * \brief Base class for all fatal errors.
 * \author tkornuta
 * \date 12.05.2011
 */
typedef error <FATAL_ERROR> fatal_error;

/*!
 * \brief Base class for all non fatal errors.
 * \author tkornuta
 * \date 12.05.2011
 */
typedef error <NON_FATAL_ERROR> non_fatal_error;

/*!
 * Macro for registration of MRROC++ system errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param DESCRIPTION Description added to the error_description error info field.
 *
 * \author tkornuta
 * \date 12.05.2011
 */
#define REGISTER_SYSTEM_ERROR(CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::system_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};

/*!
 * Macro for registration of MRROC++ fatal errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param DESCRIPTION Description added to the error_description error info field.
 *
 * \author tkornuta
 * \date 12.05.2011
 */
#define REGISTER_FATAL_ERROR(CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::fatal_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};

/*!
 * Macro for registration of MRROC++ non fatal errors.
 *
 * \param CLASS_NAME Name of the error (class name).
 * \param DESCRIPTION Description added to the error_description error info field.
 *
 * \author tkornuta
 * \date 12.05.2011
 */
#define REGISTER_NON_FATAL_ERROR(CLASS_NAME, DESCRIPTION) \
struct CLASS_NAME : virtual mrrocpp::lib::exception::non_fatal_error \
{ \
	CLASS_NAME() { *this << mrrocpp::lib::exception::error_description(DESCRIPTION); } \
	~CLASS_NAME() throw () { } \
};


} // namespace exception
} // namespace common
} // namespace mrrocpp

#endif
