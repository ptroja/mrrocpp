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

/** @addtogroup haptic
 *
 *  @{
 */

// --------------------------------------------------------------------------
// Generator trajektorii dla zadan z wodzeniem za nos w tff ze zmiana orientacji

class haptic : public continously_coordinated
{
protected:
	robot::robot *irp6ot, *irp6p;

	const lib::Homog_matrix global_base;

	lib::trajectory_description td;

public:
	int step_no;

	// konstruktor
	haptic(task::task& _mp_task, int step = 0);

	bool first_step();
	bool next_step_inside();

}; // end : class haptic

/** @} */// end of edge_following

} // namespace generator
} // namespace mp
} // namespace mrrocpp

#endif
