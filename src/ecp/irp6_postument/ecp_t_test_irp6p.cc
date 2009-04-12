// ------------------------------------------------------------------------
//   ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <map>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_g_jarosz.h"
#include "ecp/irp6_postument/ecp_t_test_irp6p.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp/irp6_postument/ecp_g_test.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {

// KONSTRUKTORY
ecp_task_test_irp6p::ecp_task_test_irp6p(configurator &_config) : ecp_task(_config)
{
}

// methods for ECP template to redefine in concrete classes
void ecp_task_test_irp6p::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_postument_robot (*this);

		// Powolanie czujnikow
	sensor_m[SENSOR_FORCE_POSTUMENT] =
		new ecp_mp::sensor::schunk (SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);

	// Konfiguracja wszystkich czujnikow
	for (std::map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}


	usleep(1000*100);

	sr_ecp_msg->message("ECP loaded");
}


void ecp_task_test_irp6p::main_task_algorithm(void)
{
	y_simple_generator ysg(*this, 8);
	ysg.sensor_m = sensor_m;

	for(;;) {
		sr_ecp_msg->message("NOWA SERIA");
		sr_ecp_msg->message("Ruch");
		sr_ecp_msg->message("Zakocz - nacisnij PULSE ECP trigger");
		ysg.Move();
	}
}

} // namespace irp6p

namespace common {

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new irp6p::ecp_task_test_irp6p(_config);
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

