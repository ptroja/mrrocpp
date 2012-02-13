#if !defined(_ECP_GEN_EDGE_FOLLOW_H)
#define _ECP_GEN_EDGE_FOLLOW_H

/*!
 * @file
 * @brief File contains ecp_generator class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */
#include "ecp_mp_g_edge_follow.h"

#include "generator/ecp/teach_in/ecp_g_teach_in.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

/**
 * @brief degrees to radians factor constant
 */
const double DEGREES_TO_RADIANS = 57.295780;

/*!
 * @brief Generator that follows unknown contour and memorizes trajectory
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup edge_follow
 */
class y_edge_follow_force : public teach_in
{
protected:

	/**
	 * @brief pose specification of memorized trajectory execute while analyzing contour
	 */
	lib::ECP_POSE_SPECIFICATION emptyps;

	/**
	 * @brief number of steps in each macrostep
	 */
	const int step_no;

	/**
	 * @brief manipualtor tool frame
	 */
	lib::Homog_matrix tool_frame;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	y_edge_follow_force(common::task::task& _ecp_task, int step);

	bool first_step();
	bool next_step();

};

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
