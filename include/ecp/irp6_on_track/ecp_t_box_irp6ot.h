//Zadanie kt�re realizuje dan� trajektori� u�ywaj�c smooth generatora

#if !defined(_ECP_T_BOX_IRP6OT_H)
#define _ECP_T_BOX_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

class ecp_t_box_irp6ot: public common::task::ecp_task {
  
  protected:
	  common::generator::ecp_smooth_generator* smoothgen;
		
	public:
		ecp_t_box_irp6ot(configurator &_config);
		~ecp_t_box_irp6ot();

		void task_initialization(void);
		void main_task_algorithm(void);
};

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

