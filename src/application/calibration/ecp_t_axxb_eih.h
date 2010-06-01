#if !defined(_ECP_T_AXXB_EIH_H)
#define _ECP_T_AXXB_EIH_H

#include <string.h>
#include <unistd.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/configurator.h"
#include "ecp/common/task/ecp_task.h"
#include "ecp_t_calib_axxb.h"
#include "gsl/gsl_vector.h"
#include "gsl/gsl_matrix.h"
#include "gsl/gsl_multimin.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class axxb_eih: public calib_axxb  {
	public:
		// KONSTRUKTORY
		axxb_eih(lib::configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
