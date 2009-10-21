/*
 * PNExecToolSpecific.cc
 *
 *  Created on: May 19, 2009
 *      Author: ptroja
 */

#include "PNExecToolSpecific.hh"

#include <cstdlib>

#include <boost/foreach.hpp>

namespace pnexec {

PNExecToolSpecific::PNExecToolSpecific(std::string _name, std::string _version)
	: name(_name), version(_version) {
}

PNExecToolSpecific::PNExecToolSpecific() {
}

PNExecToolSpecific::PNExecToolSpecific(const toolspecific_t & _toolspecific)
	: name(_toolspecific.tool()), version(_toolspecific.version()) {

	std::cout << getName() << "-" << getVersion() << ": ";

	BOOST_FOREACH(const task_t & task, _toolspecific.task()) {
		if (task.exec().present()) {
			tasks.push_back(new ExecTask(task));
		} else if (task.trajectory().present()) {
			tasks.push_back(new TrajectoryTask(task));
		}
	}

}

} // namespace
