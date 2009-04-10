#include <stdio.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_c.h"
#include "mp/mp_common_generators.h"

namespace mrrocpp {
namespace mp {
namespace task {

mp_task* return_created_mp_task (configurator &_config)
{
	return new mp_task_c(_config);
}

mp_task_c::mp_task_c(configurator &_config) : mp_task(_config)
{
}

// methods for mp template to redefine in concrete class
void mp_task_c::task_initialization(void) 
{
	sr_ecp_msg->message("MP c loaded");
}
 
void mp_task_c::main_task_algorithm(void)
{
	generator::empty empty_gen (*this); // "Pusty" generator
	empty_gen.robot_m = robot_m;
   
	// Zlecenie wykonania kolejnego makrokroku
	empty_gen.Move(); 

}


} // namespace task
} // namespace mp
} // namespace mrrocpp


