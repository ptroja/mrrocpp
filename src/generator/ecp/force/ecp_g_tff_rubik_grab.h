#if !defined(_ECP_GEN_TFF_RUBIK_GRAB_H)
#define _ECP_GEN_TFF_RUBIK_GRAB_H

/*!
 * @file
 * @brief File contains tff_rubik_grab generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "generator/ecp/ecp_g_teach_in.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// --------------------------------------------------------------------------
// Generator do lapania kostki

class tff_rubik_grab : public common::generator::generator
{
protected:
	lib::trajectory_description td;

	// do konfiguracji pracy generatora
	double goal_position, position_increment;
	unsigned int min_node_counter;
	bool both_axes_running;
	double desired_absolute_gripper_coordinate;

public:
	const int step_no;

	// konstruktor
	tff_rubik_grab(common::task::task& _ecp_task, int step = 0);

	void
			configure(double l_goal_position, double l_position_increment, unsigned int l_min_node_counter, bool l_both_axes_running =
					true);

	bool first_step();
	bool next_step();

}; // end : class ecp_tff_rubik_grab_generator


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
