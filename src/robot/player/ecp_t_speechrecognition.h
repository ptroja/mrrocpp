#if !defined(_ECP_T_SPEECHRECOGNITION_H)
#define _ECP_T_SPEECHRECOGNITION_H

#include "base/ecp/ecp_task.h"
#include "base/ecp/player/generator/ecp_g_speechrecognition.h"

namespace mrrocpp {
namespace ecp {
namespace player {
namespace task {

class speechrecognition: public common::task::task  {
protected:
	generator::speechrecognition* srg;

public:
	// KONSTRUKTORY
	speechrecognition(lib::configurator &_config);
	~speechrecognition();

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace player
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_T_SPEECHRECOGNITION_H */
