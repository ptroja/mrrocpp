#if !defined(_ECP_T_FESTIVAL_H)
#define _ECP_T_FESTIVAL_H

#include "ecp/common/ecp_task.h"
#include "ecp/festival/ecp_g_festival.h"

namespace mrrocpp {
namespace ecp {
namespace festival {
namespace task {

class base: public common::task::base  {
protected:
	generator::base* fg;

public:
	base(configurator &_config);
	~base();
	
	// methods for ECP template to redefine in concrete classes
	void task_initialization(void);
	void main_task_algorithm(void);
	
};

}
} // namespace festival
} // namespace ecp
} // namespace mrrocpp

#endif
