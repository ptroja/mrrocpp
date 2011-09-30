// ------------------------------------------------------------------------
//                                       edp.cc
//
// EDP_MASTER Effector Driver Master Process
// Driver dla robota polycrank
//
// Ostatnia modyfikacja: 13.01.2011
// -------------------------------------------------------------------------

#include <cstdio>
#include <cmath>
#include <iostream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/edp/edp_typedefs.h"
#include "base/edp/reader.h"
#include "robot/polycrank/const_polycrank.h"
#include "robot/polycrank/regulator_polycrank.h"

//#include "base/edp/edp_e_motor_driven.h"
#include "robot/polycrank/edp_e_polycrank.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

NL_regulator_polycrank::NL_regulator_polycrank(uint8_t _axis_number, uint8_t reg_no, uint8_t reg_par_no, double aa, double bb0, double bb1, double k_ff, common::motor_driven_effector &_master) :
	NL_regulator(_axis_number, reg_no, reg_par_no, aa, bb0, bb1, k_ff, _master)
{
	// Konstruktor regulatora konkretnego
	// Przy inicjacji nalezy dopilnowac, zeby numery algorytmu regulacji oraz zestawu jego parametrow byly
	// zgodne z faktycznie przekazywanym zestawem parametrow inicjujacych.
}

uint8_t NL_regulator_polycrank::compute_set_value(void)
{
	uint8_t alg_par_status = common::ALGORITHM_AND_PARAMETERS_OK;

	return alg_par_status;
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
