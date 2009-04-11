#if !defined(_ECP_T_PLAYERPOS_H)
#define _ECP_T_PLAYERPOS_H

#include "ecp/common/ecp_task.h"
#include "ecp/player/ecp_g_playerpos.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_task_playerpos: public ecp_task  {
protected:
	playerpos_generator* ppg;

public:
	// KONSTRUKTORY
	ecp_task_playerpos(configurator &_config);
	~ecp_task_playerpos();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif /* _ECP_T_PLAYERPOS_H */
