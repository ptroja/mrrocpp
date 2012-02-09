/*
 * ecp_en_labyrinth.h
 *
 * Author: enatil
 */

#ifndef ECP_EN_LABYRINTH_H
#define ECP_EN_LABYRINTH_H

#include "../../base/ecp/ecp_task.h"
#include "base/ecp/ecp_task.h"
#include "base/lib/logger.h"
#include "base/ecp/ecp_task.h"

#define UP 0
#define RIGHT 1
#define DOWN 2
#define LEFT 3


namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class ecp_en_labyrinth: public task {
	public:
		ecp_en_labyrinth(lib::configurator &_config);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif /* ECP_EN_LABYRINTH_H */
