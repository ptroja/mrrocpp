#if !defined(_ECP_T_GRASPIT_H)
#define _ECP_T_GRASPIT_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class graspit: public common::task::task {

  protected:
	  common::generator::smooth* smoothgen2;

  public:
	  graspit(lib::configurator &_config);

	  void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

