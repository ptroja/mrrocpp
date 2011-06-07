/*
 * child.cc
 *
 *  Created on: Feb 23, 2011
 *      Author: ptroja
 */

#include <cstdio>
#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>

#include "child.h"

namespace mrrocpp {
namespace lib {

child::child(configurator & config, const std::string & section)
{
	pid = config.process_spawn(section);
}

child::~child()
{
	if (kill(pid, SIGTERM) == -1) {
		perror("kill()");
	} else {
		if (waitpid(pid, NULL, 0) == -1) {
			perror("waitpid()");
		}
	}
}

}
}
