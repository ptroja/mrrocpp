//Zadanie kt�re realizuje dan� trajektori� u�ywaj�c smooth2 generatora

#if !defined(_ECP_T_SMOOTH2_TEST_H)
#define _ECP_T_SMOOTH2_TEST_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth2.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

class ecp_t_smooth2_test: public common::task::ecp_task {
  
  protected:
	  common::generator::ecp_smooth2_generator* smoothgen2;
		
	public:
		ecp_t_smooth2_test(configurator &_config);
		~ecp_t_smooth2_test();

		void task_initialization(void);
		void main_task_algorithm(void);
};

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

