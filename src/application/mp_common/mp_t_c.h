#if !defined(__MP_TASK_C_H)
#define __MP_TASK_C_H

/*!
 * @file
 * @brief File contains mp common task declaration
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup
 */

namespace mrrocpp {
namespace mp {
namespace task {

class cxx : public task
{

public:

	cxx(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	// methods fo mp template to redefine in concete class
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
