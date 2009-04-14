#include <stdio.h>
#include <unistd.h>
#include <map>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_vis_pb_eol_sac_irp6ot.h"
#include "ecp/irp6_on_track/ecp_vis_pb_eih_irp6ot.h"
#include "ecp/irp6_on_track/ecp_vis_ib_eih_irp6ot.h"
#include "ecp/irp6_on_track/ecp_vis_weights_driven_irp6ot.h"
#include "ecp_mp/ecp_mp_s_vis.h"
#include "ecp/irp6_on_track/ecp_t_vis_weights_driven_irp6ot.h"

#include "ecp_mp/ecp_mp_s_vis_sac_lx.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

// KONSTRUKTORY
vislx::vislx(lib::configurator &_config) : base(_config)
{
}

// methods for ECP template to redefine in concrete classes
void vislx::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);

	// Powolanie czujnikow

	sensor_m[lib::SENSOR_CAMERA_SA] =
		new ecp_mp::sensor::vis_sac_lx (lib::SENSOR_CAMERA_SA, "[vsp_vis]", *this); //change if SENSOR_CAMERA_SA used for nonnn recog (vsp_vis_pbeolsac)

	// Konfiguracja wszystkich czujnikow

	for (std::map <lib::SENSOR_ENUM, lib::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	usleep(1000*100);
	sr_ecp_msg->message("ECP weighted vis loaded");
}

void vislx::main_task_algorithm(void)
{
	ecp_vis_weights_driven_irp6ot ynrlg(*this, 4);
	ynrlg.sensor_m = sensor_m;

	pbeolsac = new ecp_vis_pb_eol_sac_irp6ot(*this, 4);
	ynrlg.pbeolsac=pbeolsac;

	pbeih = new ecp_vis_pb_eih_irp6ot(*this, 4);
	ynrlg.pbeih=pbeih;

	ibeih = new ecp_vis_ib_eih_irp6ot(*this, 4);
	ynrlg.pbeih=pbeih;

	for(;;) {
		sr_ecp_msg->message("NOWA SERIA");
		ynrlg.Move();
	}
}

}
} // namespace irp6ot

namespace common {
namespace task {


base* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::vislx(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

