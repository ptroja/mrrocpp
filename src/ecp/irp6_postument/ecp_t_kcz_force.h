
#if !defined(_ECP_T_KCZ_FORCE_H)
#define _ECP_T_KCZ_FORCE_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_force.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

class kcz_force: public common::task::task {

  protected:
	  common::generator::tff_nose_run* nose_run;

  public:
	  kcz_force(lib::configurator &_config);

	  void main_task_algorithm(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif

