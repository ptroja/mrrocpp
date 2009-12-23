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

class acquisition: public ecp_sub_task {
	public:
		// KONSTRUKTORY
		acquisition(task &_ecp_t);

		// methods for ECP template to redefine in concrete classes
		void main_task_algorithm(void);

		virtual void write_data(std::string K_fp, std::string k_fp, std::string M_fp, std::string m_fp, int number_of_measures) = 0;
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
