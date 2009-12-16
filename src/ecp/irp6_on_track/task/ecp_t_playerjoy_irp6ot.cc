// ------------------------------------------------------------------------
//   task/ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------


#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"
#include "ecp_mp/sensor/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/common/generator/ecp_g_pjg.h"
#include "ecp_mp/transmitter/ecp_mp_tr_player.h"
#include "ecp/irp6_on_track/task/ecp_t_playerjoy_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

// KONSTRUKTORY
playerjoy::playerjoy(lib::configurator &_config) : task(_config)
{
    ecp_m_robot = new robot (*this);

    sr_ecp_msg->message("ECP loaded");

    transmitter_m[ecp_mp::transmitter::TRANSMITTER_PLAYER] =
        new ecp_mp::transmitter::player (ecp_mp::transmitter::TRANSMITTER_PLAYER, "[transmitter_player]", *this,
                                "192.168.1.68", 6665, "joystick", 0, 'r');

    pjg = new common::generator::playerjoy(*this, 8);
    pjg->transmitter_m = transmitter_m;
}

void playerjoy::main_task_algorithm(void)
{
	for(;;)
	{
		pjg->Move();
	}
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::playerjoy(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


