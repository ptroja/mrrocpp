// -------------------------------------------------------------------------
//
// Definicje struktur danych i metod dla procesow MP - generatory silowe
//
// -------------------------------------------------------------------------

#if !defined(__MP_GEN_HAPTIC_H)
#define __MP_GEN_HAPTIC_H

#include "lib/mrmath/mrmath.h"

#include "mp/generator/mp_generator.h"
#include "mp/robot/mp_robot.h"

namespace mrrocpp {
namespace mp {
namespace generator {

/** @addtogroup ball
 *
 *  @{
 */

// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class ball : public generator
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
	int step_no;
	//     double delta[6];

	// konstruktor
	ball(task::task& _mp_task, int step = 0);

	void configure(bool l_irp6ot_con, bool l_irp6p_con);

	virtual bool first_step();
	virtual bool next_step();
}; // end : class ball

/** @} */// end of edge_following

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif
