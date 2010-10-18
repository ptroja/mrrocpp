#if !defined(_ECP_MAIN_ERROR_H)
#define  _ECP_MAIN_ERROR_H

/*!
 * @file
 * @brief File contains ECP_main_error class declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/lib/com_buf.h"

namespace mrrocpp {
namespace ecp {
namespace common {

/*!
 * @brief ECP main error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup ecp
 */
class ECP_main_error
{
public:
	/**
	 * @brief error class (type)
	 */
	const lib::error_class_t error_class;

	/**
	 * @brief error number
	 */
	const uint64_t error_no;

	/**
	 * @brief constructor
	 * @param err_cl error class
	 * @param err_no error number
	 */
	ECP_main_error(lib::error_class_t err_cl, uint64_t err_no);
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_MAIN_ERROR_H */
