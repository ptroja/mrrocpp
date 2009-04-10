#if !defined(__MP_TASK_C_H)
#define __MP_TASK_C_H

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace task {

class cxx: public base
{
	
public:
	
	cxx(configurator &_config);

	// methods fo mp template to redefine in concete class
	void task_initialization(void);
	void main_task_algorithm(void);
  
};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
