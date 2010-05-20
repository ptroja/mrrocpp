// ------------------------------------------------------------------------
//                                  transformer_error.cc
//
//
// Ostatnia modyfikacja: styczen 2009
// -------------------------------------------------------------------------

#include <stdint.h>

#include "lib/exception.h"

namespace mrrocpp {
namespace lib {
namespace exception {

Error_base::~Error_base() throw ()
{
}

System_error::~System_error() throw ()
{
}

Fatal_error::Fatal_error(uint64_t err_no_0, uint64_t err_no_1) :
	error0(err_no_0), error1(err_no_1)
{
}

Fatal_error::~Fatal_error() throw ()
{
}

NonFatal_error::NonFatal_error()
{
}

NonFatal_error::~NonFatal_error() throw ()
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

