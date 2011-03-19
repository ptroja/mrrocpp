#if !defined(_ECP_T_AXZB_EIH_H)
#define _ECP_T_AXZB_EIH_H

#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"
#include "base/lib/configurator.h"
#include "base/ecp/ecp_task.h"
#include "ecp_t_calib_axzb.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class axzb_eih: public calib_axzb  {
	public:
		// KONSTRUKTORY
		axzb_eih(lib::configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
