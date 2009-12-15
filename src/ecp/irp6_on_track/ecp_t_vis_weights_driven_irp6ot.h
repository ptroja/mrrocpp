#if !defined(_ECP_T_WEIGHTS_DRIVEN_IRP6OT_H)
#define _ECP_T_WEIGHTS_DRIVEN_IRP6OT_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/irp6_on_track/ecp_vis_pb_eol_sac_irp6ot.h"
#include "ecp/irp6_on_track/ecp_vis_pb_eih_irp6ot.h"
#include "ecp/irp6_on_track/ecp_vis_ib_eih_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class vislx: public common::task::task  {
	ecp_vis_pb_eol_sac_irp6ot* pbeolsac;
	ecp_vis_pb_eih_irp6ot* pbeih;
	ecp_vis_ib_eih_irp6ot* ibeih;

	public:
		// KONSTRUKTORY
		vislx(lib::configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
