#if !defined(__MP_T_edge_follow_MR_H)
#define __MP_T_edge_follow_MR_H

/*!
 * @file
 * @brief File contains edge_follow_mr mp_task class declaration of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

namespace mrrocpp {
namespace mp {
namespace task {

/*!
 * @brief Task that executes motion of manipulator and its gripper to follow an unknown contour
 *
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup edge_follow
 */
class edge_follow_mr : public task
{
protected:

public:

	/**
	 * @brief Constructor
	 * @param _config configurator object reference.
	 */
	edge_follow_mr(lib::configurator &_config);

	void create_robots(void);

	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
