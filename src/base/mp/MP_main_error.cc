#include "base/mp/MP_main_error.h"

namespace mrrocpp {
namespace mp {
namespace common {

MP_main_error::MP_main_error(lib::error_class_t err0, uint64_t err1) :
	error_class(err0), error_no(err1)
{
}

} // namespace common
} // namespace mp
} // namespace mrrocpp
