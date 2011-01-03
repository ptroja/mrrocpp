#if !defined(_MP_MAIN_ERROR_H)
#define _MP_MAIN_ERROR_H

/*!
 * @file
 * @brief File contains MP_main_error class declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup mp
 */

#include <map>

#include "base/lib/com_buf.h"

namespace mrrocpp {
namespace mp {
namespace robot {
class robot;
}

namespace common {

/*!
 * @brief MP main error handling class
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup mp
 */
class MP_main_error
{ // Klasa obslugi bledow poziomie MP
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
	 * @param err0 error class
	 * @param err1 error number
	 */
	MP_main_error(lib::error_class_t err0, uint64_t err1);

};
// ---------------------------------------------------------------

} // namespace common

} // namespace mp
} // namespace mrrocpp


#endif
