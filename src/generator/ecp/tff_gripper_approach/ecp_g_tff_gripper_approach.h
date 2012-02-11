#if !defined(_ECP_GEN_TFF_GRIPPER_APPROACH_H)
#define _ECP_GEN_TFF_GRIPPER_APPROACH_H

/*!
 * @file
 * @brief File contains tff_gripper_approach generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "ecp_mp_g_tff_gripper_approach.h"
#include "base/ecp/ecp_generator.h"

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
	double force_level;

public:
	const int step_no;

	// konstruktor
	tff_gripper_approach(common::task::task& _ecp_task, int step = 0);

	void configure(double l_speed, unsigned int l_motion_time, double l_force_level);

	bool first_step();
	bool next_step();

	void conditional_execution();

};
// end : class ecp_tff_gripper_approach_generator

}// namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
