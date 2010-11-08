/*
 * Worker.hh
 *
 *  Created on: Nov 1, 2009
 *      Author: ptroja
 */

#ifndef WORKER_HH_
#define WORKER_HH_

#include <map>
#include "lib/impconst.h"

namespace pnexec {

typedef std::map<mrrocpp::lib::robot_name_t, class Place *> workers_t;

}

#endif /* WORKER_HH_ */
