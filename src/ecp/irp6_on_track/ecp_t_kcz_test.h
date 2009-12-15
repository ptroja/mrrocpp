//Zadanie kt�re realizuje dan� trajektori� u�ywaj�c smooth2 generatora

#if !defined(_ECP_T_KCZ_TEST_H)
#define _ECP_T_KCZ_TEST_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth2.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class kcz_test: public common::task::task {

  protected:
	  common::generator::smooth2* smoothgen2;
	  double kw_bok;

  public:
	  kcz_test(lib::configurator &_config);

	  void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

