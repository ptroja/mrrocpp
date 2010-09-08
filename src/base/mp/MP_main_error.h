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

// ---------------------------------------------------------------
class MP_main_error
{ // Klasa obslugi bledow poziomie MP
public:
	const lib::error_class_t error_class;
	const uint64_t error_no;

	MP_main_error(lib::error_class_t err0, uint64_t err1);

};
// ---------------------------------------------------------------

} // namespace common

} // namespace mp
} // namespace mrrocpp


#endif
