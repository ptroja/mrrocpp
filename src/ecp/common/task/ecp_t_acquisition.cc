#include "ecp/common/task/ecp_t_acquisition.h"

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

int acquisition::write_data(void)
{
}

//task* return_created_ecp_task (lib::configurator &_config)
//{
//	return new calibration(_config);
//}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
