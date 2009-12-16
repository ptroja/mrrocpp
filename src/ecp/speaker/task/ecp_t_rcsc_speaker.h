#if !defined(_ECP_T_RCSC_SPEAKER_H)
#define _ECP_T_RCSC_SPEAKER_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_t.h"
#include "ecp/speaker/generator/ecp_g_speak.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {
namespace task {

class rcsc: public common::task::task  {
protected:
	common::generator::transparent* gt;
	generator::speaking* speak;

public:
	// KONSTRUKTORY
	rcsc(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);

};

}
} // namespace speaker
} // namespace ecp
} // namespace mrrocpp

#endif
