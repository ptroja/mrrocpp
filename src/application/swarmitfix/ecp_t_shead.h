#if !defined(_ECP_T_SHEAD_SWARMITFIX_H)
#define _ECP_T_SHEAD_SWARMITFIX_H

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_g_transparent.h"


namespace mrrocpp {
namespace ecp {
namespace shead {
namespace task {

class swarmitfix: public common::task::task
{
protected:
    //generatory
	common::generator::transparent* gt;
	//common::generator::smooth* sg;
	common::generator::sleep* g_sleep;
	generator::head_soldify* g_head_soldify;
	generator::head_desoldify* g_head_desoldify;
	generator::head_vacuum_on* g_head_vacuum_on;
	generator::head_vacuum_off* g_head_vacuum_off;

public:
    // KONSTRUKTORY
    swarmitfix(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

#endif
