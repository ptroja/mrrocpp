// -------------------------------------------------------------------------
//
// Definicje struktur danych i metod dla procesow MP - generatory silowe
//
// -------------------------------------------------------------------------

#if !defined(__MP_GEN_HAPTIC_H)
#define __MP_GEN_HAPTIC_H

#include "lib/mrmath/mrmath.h"

#include "base/mp/mp_generator.h"
#include "base/mp/mp_robot.h"

#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> //include all types plus i/o

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
	robot::irp6ot_m *irp6ot;
	robot::irp6p_m *irp6p;

	//! initial speed factor of trajectory
	double speedup;

	//! trajectory speeed up factor
	const double speedup_factor;

	//! startup positions
	lib::Homog_matrix irp6ot_start, irp6p_start;

	void setup_command(robot::robot & robot);

	//! Clock value at generator's start
	boost::posix_time::ptime start_clk;

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
