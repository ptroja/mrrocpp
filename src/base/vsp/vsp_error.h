/*!
 * @file
 * @brief File containing class that represents errors (exceptions) passed in VSPs.
 * @date 30.11.2006
 * @author tkornuta <tkornuta@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup VSP
 */

#if !defined(_VSP_ERROR_H)
#define _VSP_ERROR_H

#include <exception>

namespace mrrocpp {
namespace vsp {
namespace common {

/*!
 *
 * @brief Class representing the exceptions thrown and handled in VSP.
 * @author tkornuta
 */
class vsp_error : public std::exception
{
public:
	/** Error class. */
	const lib::error_class_t error_class;

	/** Error number. */
	const uint64_t error_no;

	/*!
	 * Constructor.
	 * @param err_cl Error class.
	 * @param err_no Error number.
	 */
	vsp_error(lib::error_class_t err_cl, uint64_t err_no) :
		std::exception(), error_class(err_cl), error_no(err_no)
	{
	}
};

} // namespace common
} // namespace vsp
} // namespace mrrocpp

#endif
