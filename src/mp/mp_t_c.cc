#include <stdio.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/mp_t_c.h"
#include "mp/mp_common_generators.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task (lib::configurator &_config)
{
	return new cxx(_config);
}

cxx::cxx(lib::configurator &_config) : task(_config)
{
}

void cxx::main_task_algorithm(void)
{
	generator::empty empty_gen (*this); // "Pusty" generator
	empty_gen.robot_m = robot_m;

	// Zlecenie wykonania kolejnego makrokroku
	empty_gen.Move();
}


} // namespace task
} // namespace mp
} // namespace mrrocpp


