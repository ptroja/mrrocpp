#if !defined(_ECP_T_SMB_SWARM_DEMO_SINGLE_AGENT_H)
#define _ECP_T_SMB_SWARM_DEMO_SINGLE_AGENT_H

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_g_transparent.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace task {

class swarmitfix : public common::task::task
{
protected:
	//generatory
	common::generator::transparent* gt;
	common::generator::sleep* g_sleep;
	generator::legs_command* g_legs_command;
	generator::external_epos_command* g_external_epos_command;

public:
	// KONSTRUKTORY
	swarmitfix(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
