#if !defined(_ECP_T_SPEAKER_H)
#define _ECP_T_SPEAKER_H

#include "ecp/common/ecp_task.h"
#include "ecp/speaker/ecp_g_speak.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {
namespace task {

class ecp_task_speaker: public common::task::base  {
protected:
	generator::speaking_generator* speak;

public:
	// KONSTRUKTORY
	ecp_task_speaker(configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);

};

}
} // namespace speaker
} // namespace ecp
} // namespace mrrocpp

#endif
