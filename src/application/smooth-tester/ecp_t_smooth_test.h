//Zadanie kt�re realizuje dan� trajektori� u�ywaj�c smooth generatora

#if !defined(_ECP_T_smooth_TEST_H)
#define _ECP_T_smooth_TEST_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

class smooth_test: public common::task::task {

  protected:
	  common::generator::smooth* smoothgen2;

	public:
		smooth_test(lib::configurator &_config);

		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

