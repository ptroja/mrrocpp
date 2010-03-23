#include <stdio.h>
#include <unistd.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "mp/mp.h"
#include "mp/task/mp_t_c.h"
#include "mp/generator/mp_g_common.h"

#include <boost/foreach.hpp>
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

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
	{

		 robot_node.second->ecp_reply_package.reply = lib::ECP_ACKNOWLEDGE;


	}

	generator::empty empty_gen (*this); // "Pusty" generator
	empty_gen.robot_m = robot_m;

	// Zlecenie wykonania kolejnego makrokroku
	empty_gen.Move();
}


} // namespace task
} // namespace mp
} // namespace mrrocpp


