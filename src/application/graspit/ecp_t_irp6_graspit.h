#if !defined(_ECP_T_GRASPIT_H)
#define _ECP_T_GRASPIT_H

#include "base/ecp/ecp_task.h"
#include "ecp_mp_tr_graspit.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

/*!
 * \class irp6_grasp
 * \brief Class for getting commands from MP and moving IRp6
 *
 * \author kczajkow
 * \date Jan 05, 2010
 */
class irp6_grasp: public common::task::task {

  private:
	  generator::constant_velocity* cvgenjoint;;

  public:
	  irp6_grasp(lib::configurator &_config);

	  void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

