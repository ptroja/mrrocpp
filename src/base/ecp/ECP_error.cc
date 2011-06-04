/*!
 * @file
 * @brief File contains ecp base generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include "base/ecp/ECP_error.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

ECP_error::ECP_error(lib::error_class_t err_cl, uint64_t err_no, uint64_t err0, uint64_t err1) :
	error_class(err_cl), error_no(err_no)
{
	error.error0 = err0;
	error.error1 = err1;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp


