#if !defined(__MP_T_edge_follow_MR_H)
#define __MP_T_edge_follow_MR_H

/*!
 * @file mp_t_edge_follow_mr.h
 * @brief File contains edge_follow_mr mp_task class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

namespace mrrocpp {
namespace mp {
namespace task {

class edge_follow_mr : public task
{
protected:

public:

	edge_follow_mr(lib::configurator &_config);

	// methods for mp template
	void main_task_algorithm(void);

};

/** @} */// end of edge_follow

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
