#include "ecp/common/ECP_main_error.h"

namespace mrrocpp {
namespace ecp {
namespace common {

ECP_main_error::ECP_main_error ( lib::ERROR_CLASS err_cl, uint64_t err_no)
: error_class(err_cl), error_no(err_no)
{}

} // namespace common
} // namespace ecp
} // namespace mrrocpp
