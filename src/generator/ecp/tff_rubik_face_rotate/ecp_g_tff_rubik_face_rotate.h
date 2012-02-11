#if !defined(_ECP_GEN_RUBIK_FACE_ROTATE_H)
#define _ECP_GEN_RUBIK_FACE_ROTATE_H

/*!
 * @file
 * @brief File contains tff_rubik_face_rotate generator declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include "ecp_mp_g_tff_rubik_face_rotate.h"
#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// --------------------------------------------------------------------------
// Generator do obracania sciany kostki

class tff_rubik_face_rotate : public common::generator::generator
{
protected:
	lib::trajectory_description td;

	// do konfiguracji pracy generatora

	double stored_gamma, turn_angle;
	bool range_change;

public:
	const int step_no;

	// konstruktor
	tff_rubik_face_rotate(common::task::task& _ecp_task, int step = 0);

	void configure(double l_turn_angle);

	bool first_step();
	bool next_step();

	void conditional_execution();

};
// end : class ecp_tff_rubik_face_rotate_generator

}// namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
