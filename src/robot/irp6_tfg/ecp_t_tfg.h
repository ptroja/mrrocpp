#if !defined(_ECP_T_TFG_H)
#define _ECP_T_TFG_H

#include "ecp/ecp_task.h"
#include "ecp/generator/ecp_g_force.h"
#include "robot/irp6_tfg/ecp_g_tfg.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {




class tfg: public common::task::task {
protected:
	//generatory
	generator::tfg* tfgg;
	bool save_activated;

public:
	// KONSTRUKTORY
	tfg(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
