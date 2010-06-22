#if !defined(_ECP_T_RCSC_SPEAKER_H)
#define _ECP_T_RCSC_SPEAKER_H

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_g_transparent.h"
#include "robot/speaker/ecp_g_speak.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {
namespace task {

class rcsc : public common::task::task
{
protected:
	common::generator::transparent* gt;
	generator::speaking* speak;

public:
	// KONSTRUKTORY
	rcsc(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);

};

}
} // namespace speaker
} // namespace ecp
} // namespace mrrocpp

#endif
