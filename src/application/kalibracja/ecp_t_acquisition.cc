#include <string>
#include <iostream>
#include "application/kalibracja/ecp_t_acquisition.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
acquisition::acquisition(lib::configurator &_config) : task(_config)
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
