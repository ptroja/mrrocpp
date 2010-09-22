#include <string>
#include <iostream>
#include "ecp_st_acquisition.h"
#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

// KONSTRUKTORY
acquisition::acquisition(task::task &_ecp_t) :
	sub_task(_ecp_t)
{
}

void acquisition::main_task_algorithm(void)
{
}

//task* return_created_ecp_task (lib::configurator &_config)
//{
//	return new acquisition(_config);
//}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
