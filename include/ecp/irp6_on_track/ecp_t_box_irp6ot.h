//Zadanie kt�re realizuje dan� trajektori� u�ywaj�c smooth generatora

#if !defined(_ECP_T_BOX_IRP6OT_H)
#define _ECP_T_BOX_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class box: public common::task::base {
  
  protected:
	  common::generator::smooth* smoothgen;
		
	public:
		box(lib::configurator &_config);
		~box();

		void task_initialization(void);
		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

