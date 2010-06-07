/*
 * mp_g_pnml.cc
 *
 *  Created on: Oct 20, 2009
 *      Author: ptroja
 */

#include <boost/foreach.hpp>

#include "PNExec/Net.hh"
#include "mp_g_pnml.h"

namespace mrrocpp {
namespace mp {
namespace generator {

pnmlExecutor::pnmlExecutor(task::task& _mp_task, pnexec::Net & _net)
	: generator(_mp_task), pnmlNet(_net)
{
}

bool pnmlExecutor::first_step() {
	// we do not want to send any commands to the robots at the beginning
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
		robot_node.second->communicate = false;
	}
	return true;
}

bool pnmlExecutor::next_step() {
	std::cerr << "pnmlExecutor::next_step()" << std::endl;

	// by default there is no need to send the commands to the robot
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m) {
		robot_node.second->communicate = false;
	}

	// wait_for_ECP_pulse =
	pnmlNet.ExecuteStep(robot_m);
	return true;
}

}
}
}
