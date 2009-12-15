#if !defined(_ECP_T_PLAYERPOS_H)
#define _ECP_T_PLAYERPOS_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/player/ecp_g_playerpos.h"

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
	void main_task_algorithm(void);
};

}
} // namespace player
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_T_PLAYERPOS_H */
