#if !defined(_ECP_T_SPEECHRECOGNITION_H)
#define _ECP_T_SPEECHRECOGNITION_H

#include "ecp/common/ecp_task.h"
#include "ecp/player/ecp_g_speechrecognition.h"

namespace mrrocpp {
namespace ecp {
namespace player {

class ecp_task_speechrecognition: public common::task::ecp_task  {
protected:
	speechrecognition_generator* srg;

public:
	// KONSTRUKTORY
	ecp_task_speechrecognition(configurator &_config);
	~ecp_task_speechrecognition();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

} // namespace player
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_T_SPEECHRECOGNITION_H */
