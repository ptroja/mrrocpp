#if !defined(_ECP_T_PAWEL_H)
#define _ECP_T_PAWEL_H

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_pawel.h"
// Konfigurator.
#include "lib/configurator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class pawel: public common::task::task  {
	protected:
		//	ecp_tff_nose_run_generator* nrg;
		//	ecp_teach_in_generator* tig;
		generator::pawel* pg;
		double x,y,z;

	public:
		// KONSTRUKTORY
		pawel(lib::configurator &_config);

		// methods for ECP template to redefine in concrete classes
		void task_initialization(void);
		void main_task_algorithm(void);
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
