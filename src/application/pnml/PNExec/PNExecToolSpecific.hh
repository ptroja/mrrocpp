/*
 * PNExecToolSpecific.hh
 *
 *  Created on: May 19, 2009
 *      Author: ptroja
 */

#ifndef PNEXECTOOLSPECIFIC_HH_
#define PNEXECTOOLSPECIFIC_HH_

#include <iostream>
#include <string>

#include <boost/ptr_container/ptr_list.hpp>

#include "Task.hh"

#include "pipepnml.hxx"

namespace pnexec {

class PNExecToolSpecific {
	private:
		std::string name;
		std::string version;

	public:
		typedef boost::ptr_list<Task> tasklist_t;

		PNExecToolSpecific(std::string _name, std::string _version);
		PNExecToolSpecific(const toolspecific_t & _toolspecific);
		PNExecToolSpecific(void);

		std::string getName() const {
			return name;
		}

		std::string getVersion() const {
			return version;
		}

		// tasks to execute in this place
		tasklist_t tasks;
};

} // namespace

#endif /* PNEXECTOOLSPECIFIC_HH_ */
