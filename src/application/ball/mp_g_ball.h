// -------------------------------------------------------------------------
//
// Definicje struktur danych i metod dla procesow MP - generatory silowe
//
// -------------------------------------------------------------------------

#if !defined(__MP_GEN_HAPTIC_H)
#define __MP_GEN_HAPTIC_H

#include "base/lib/mrmath/mrmath.h"

#include "base/mp/generator/mp_g_continously_coordinated.h"

namespace mrrocpp {
namespace mp {
namespace generator {

/** @addtogroup ball
 *
 *  @{
 */

// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class ball : public continously_coordinated
{
private:
	robot::robot *irp6ot, *irp6p;
	//lib::sensor *vsp_force_irp6ot, *vsp_force_irp6p;

	// do konfiguracji pracy generatora
	bool irp6ot_con, irp6p_con;
	lib::Homog_matrix global_base;

	//! initial speed factor of trajectory
	double speedup;

	//! trajectory speeed up factor
	const double speedup_factor;

	//! startup positions
	lib::Homog_matrix irp6ot_start, irp6p_start;

	void setup_command(robot::robot & robot);

public:
	// konstruktor
	ball(task::task& _mp_task);

	void configure(bool l_irp6ot_con, bool l_irp6p_con);

	bool first_step();
	bool next_step_inside();
}; // end : class ball

/** @} */// end of edge_following

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif
