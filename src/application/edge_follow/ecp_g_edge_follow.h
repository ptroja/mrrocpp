#if !defined(_ECP_GEN_edge_follow_H)
#define _ECP_GEN_edge_follow_H

/*!
 * @file
 * @brief File contains ecp_generator class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include "generator/ecp/ecp_g_teach_in.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

const double DEGREES_TO_RADIANS = 57.295780;

class y_edge_follow_force : public teach_in
{
protected:

	lib::ECP_POSE_SPECIFICATION emptyps;
	lib::trajectory_description td;
	const int step_no;
	double delta[6];
	lib::Homog_matrix basic_rot_frame;
	lib::Homog_matrix tool_frame;
	lib::Homog_matrix ex_rot_frame;

public:

	// konstruktor
	y_edge_follow_force(common::task::task& _ecp_task, int step);

	virtual bool first_step();

	virtual bool next_step();

}; // end:


} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
