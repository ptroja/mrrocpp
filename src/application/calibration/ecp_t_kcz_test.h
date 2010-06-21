#if !defined(_ECP_T_KCZ_TEST_H)
#define _ECP_T_KCZ_TEST_H

#include "ecp/ecp_task.h"
#include "generator/ecp/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class kcz_test: public common::task::task {

  protected:
	  common::generator::smooth* smoothgen2;

  public:
	  kcz_test(lib::configurator &_config);

	  void main_task_algorithm(void);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

