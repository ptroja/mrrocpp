#if !defined(_ECP_GEN_TFF_GRIPPER_APPROACH_H)
#define _ECP_GEN_TFF_GRIPPER_APPROACH_H

/*!
 * @file
 * @brief File contains tff_gripper_approach generator declaration
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
// Generator do nasuniecia chwytaka na kostke

class tff_gripper_approach : public common::generator::generator
{
protected:

	lib::trajectory_description td;

	// do konfiguracji pracy generatora
	double speed;
	unsigned int motion_time;

public:
	const int step_no;

	// konstruktor
	tff_gripper_approach(common::task::task& _ecp_task, int step = 0);

	void configure(double l_speed, unsigned int l_motion_time);

	bool first_step();
	bool next_step();

}; // end : class ecp_tff_gripper_approach_generator


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
