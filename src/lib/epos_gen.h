// ------------------------------------------------------------------------
// Plik:				mis_fun.h
// Opis:			miscellaneous functions
// ------------------------------------------------------------------------

#ifndef __EPOS_GEN_LIB_H
#define __EPOS_GEN_LIB_H

#include "lib/data_port_headers/epos.h"

namespace mrrocpp {
namespace lib {

void compute_epos_command(const epos_gen_parameters& input,
		epos_low_level_command& output);

} // namespace lib
} // namespace mrrocpp

#endif
