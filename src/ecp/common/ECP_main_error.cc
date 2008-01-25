#include "ecp/common/ECP_main_error.h"

ECP_main_error::ECP_main_error ( uint64_t err_cl, uint64_t err_no)
: error_class(err_cl), error_no(err_no)
{}
