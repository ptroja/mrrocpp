#include <string.h>
#include <unistd.h>

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_g_smooth2.h"
#include "ecp/common/ecp_t_acquisition.h"

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
