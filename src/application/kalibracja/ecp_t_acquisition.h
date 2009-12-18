#if !defined(_ECP_T_ACQUISITION_H)
#define _ECP_T_ACQUISITION_H

#include <string.h>
#include <unistd.h>

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/configurator.h"
#include "ecp/common/task/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class acquisition: public common::task::task  {
	protected:
		int write_data(void);

	public:
		// KONSTRUKTORY
		acquisition(lib::configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
