//Zadanie kt�re realizuje dan� trajektori� u�ywaj�c smooth2 generatora

#if !defined(_ECP_T_GRAB_CUBE_IRP6OT_H)
#define _ECP_T_GRAB_CUBE_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_g_smooth2.h"
#include "ecp/irp6_on_track/ecp_vis_ib_eih_object_tracker_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class grab_cube: public common::task::task {

  protected:
	  common::generator::smooth2* smoothgen2;
	  ecp_vis_ib_eih_object_tracker_irp6ot* tracker;

	public:
		grab_cube(lib::configurator &_config);

		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif

