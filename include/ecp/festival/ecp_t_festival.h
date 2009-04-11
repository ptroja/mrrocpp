#if !defined(_ECP_T_FESTIVAL_H)
#define _ECP_T_FESTIVAL_H

#include "ecp/common/ecp_task.h"
#include "ecp/festival/ecp_g_festival.h"

namespace mrrocpp {
namespace ecp {
namespace common {

class ecp_task_festival: public ecp_task  {
protected:
	festival_generator* fg;

public:
	ecp_task_festival(configurator &_config);
	~ecp_task_festival();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
