#if !defined(_ECP_T_PLAYERPOS_H)
#define _ECP_T_PLAYERPOS_H

#include "base/ecp/ecp_task.h"
#include "base/ecp/player/generator/ecp_g_playerpos.h"

namespace mrrocpp {
namespace ecp {
namespace player {
namespace task {

class playerpos: public common::task::task  {
protected:
	generator::playerpos* ppg;

public:
	// KONSTRUKTORY
	playerpos(lib::configurator &_config);
	~playerpos();

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace player
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_T_PLAYERPOS_H */
