/*!
 * @file exception.h
 * @brief Exception declarations.
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#include <stdint.h>

#include "base/lib/exception.h"

namespace mrrocpp {
namespace lib {
namespace exception {

Fatal_error::Fatal_error(uint64_t err_no_0, uint64_t err_no_1) :
	error0(err_no_0), error1(err_no_1)
{
}

NonFatal_error_1::NonFatal_error_1(uint64_t err_no) :
	error(err_no)
{
}

NonFatal_error_2::NonFatal_error_2(uint64_t err_no) :
	error(err_no)
{
}

NonFatal_error_3::NonFatal_error_3(uint64_t err_no) :
	error(err_no)
{
}

NonFatal_error_4::NonFatal_error_4(uint64_t err_no) :
	error(err_no)
{
}

} // namespace exception
} // namespace common
} // namespace mrrocpp

