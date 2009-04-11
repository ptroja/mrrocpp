//Zadanie kt�re realizuje dan� trajektori� u�ywaj�c smooth generatora

#if !defined(_ECP_T_BOX_IRP6OT_H)
#define _ECP_T_BOX_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_t_box_irp6ot: public ecp_task {
  
  protected:
		ecp_smooth_generator* smoothgen;
		
	public:
		ecp_t_box_irp6ot(configurator &_config);
		~ecp_t_box_irp6ot();

		void task_initialization(void);
		void main_task_algorithm(void);
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif

