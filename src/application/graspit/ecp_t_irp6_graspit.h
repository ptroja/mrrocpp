#if !defined(_ECP_T_GRASPIT_H)
#define _ECP_T_GRASPIT_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp_mp_tr_graspit.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class irp6_grasp: public common::task::task {

  private:
	  common::generator::smooth* smoothgen2;

  public:
	  irp6_grasp(lib::configurator &_config);

	  void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

