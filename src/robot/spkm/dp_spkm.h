#if !defined(__SPKM_DATA_PORT_H)
#define __SPKM_DATA_PORT_H

/*!
 * @file
 * @brief File contains data port communication structures for SwarmItFix Parallel Kinematic Machine
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spkm
 */

#include "robot/epos/dp_epos.h"

namespace mrrocpp {
namespace lib {
namespace spkm {

/*!
 * @brief SwarmItFix arallel Kinematic Machine total number of servos
 * @ingroup spkm
 */
const int NUM_OF_SERVOS = 7;

/*!
 * @brief SwarmItFix arallel Kinematic Machine mp to ecp command
 * @ingroup spkm
 */
struct mp_to_ecp_parameters
{
	epos::EPOS_GEN_PROFILE motion_type;
	epos::mp_to_ecp_cubic_trapezoidal_parameters cubic_trapezoidal[NUM_OF_SERVOS];
	lib::frame_tab goal_frame;
	double m;
	double tau;
	bool guarded_move_manipulator;
};

} // namespace spkm
} // namespace lib
} // namespace mrrocpp

#endif
