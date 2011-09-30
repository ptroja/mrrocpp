/*!
 * @file
 * @brief File contains ECP_main_error class definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/ecp/ECP_main_error.h"

namespace mrrocpp {
namespace ecp {
namespace common {

ECP_main_error::ECP_main_error(lib::error_class_t err_cl, uint64_t err_no) :
	error_class(err_cl), error_no(err_no)
{
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp
