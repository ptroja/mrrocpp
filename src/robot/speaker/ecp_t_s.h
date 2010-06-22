#if !defined(_ECP_T_SPEAKER_H)
#define _ECP_T_SPEAKER_H

#include "base/ecp/ecp_task.h"
#include "robot/speaker/ecp_g_speak.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {
namespace task {

class speaking : public common::task::task
{
protected:
	generator::speaking* speak;

public:
	// KONSTRUKTORY
	speaking(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);
};

}
} // namespace speaker
} // namespace ecp
} // namespace mrrocpp

#endif
