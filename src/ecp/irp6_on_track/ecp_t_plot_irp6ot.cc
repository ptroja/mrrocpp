#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/common/ecp_g_plot.h"
#include "ecp/irp6_on_track/ecp_t_plot_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


// KONSTRUKTORY
plot::plot(lib::configurator &_config) : task(_config)
{
}


// methods for ECP template to redefine in concrete classes
void plot::task_initialization(void)
{
    ecp_m_robot = new robot (*this);

    sr_ecp_msg->message("ECP loaded");
}

void plot::main_task_algorithm(void)
{
	common::generator::y_simple ysg(*this, 8);
    ysg.sensor_m = sensor_m;

    ysg.Move();
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::plot(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

