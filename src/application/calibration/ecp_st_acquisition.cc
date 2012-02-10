#include <string>
#include <iostream>
#include "ecp_st_acquisition.h"
#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace subtask {

// KONSTRUKTORY
acquisition::acquisition(task::task &_ecp_t) :
	subtask(_ecp_t)
{
}

void acquisition::main_task_algorithm(void)
{
}

//task_base* return_created_ecp_task (lib::configurator &_config)
//{
//	return new acquisition(_config);
//}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
