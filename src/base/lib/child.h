/*
 * child.h
 *
 *  Created on: Feb 23, 2011
 *      Author: ptroja
 */

#ifndef CHILD_H_
#define CHILD_H_

#include <unistd.h>

#include "configurator.h"

namespace mrrocpp {
namespace lib {

//! Object-oriented container for a child process
class child {
private:
	//! Created process identifier
	pid_t pid;
public:
	//! Constructor
	//! @brief creates a child process
	//! @param config a configurator object
	//! @param section section with a child process to create
	child(configurator & config, const std::string & section);

	//! Destructor
	//! @brief kill and waint until process has terminated
	~child();
};

}
}

#endif /* CHILD_H_ */
