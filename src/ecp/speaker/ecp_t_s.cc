// ------------------------------------------------------------------------
//   ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------


#include <stdio.h>
#include <unistd.h>
#include <map>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/speaker/ecp_r_speaker.h"
#include "ecp/speaker/ecp_t_s.h"
#include "ecp_mp/ecp_mp_s_mic.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {
namespace task {

// KONSTRUKTORY
speaking::speaking(lib::configurator &_config) : task(_config)
{
    speak = NULL;
}

// methods for ECP template to redefine in concrete classes
void speaking::task_initialization(void)
{
    ecp_m_robot = new ecp_speaker_robot (*this);

    sensor_m[lib::SENSOR_MIC] =
        new ecp_mp::sensor::mic(lib::SENSOR_MIC, "[vsp_mic]", *this);

    // Konfiguracja wszystkich czujnikow
    for (ecp_mp::sensor_map::iterator sensor_m_iterator = sensor_m.begin();
            sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
    {
        sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
        sensor_m_iterator->second->configure_sensor();
    }

    usleep(1000*100);

    speak = new generator::speaking (*this, 8);
    speak->sensor_m = sensor_m;

    sr_ecp_msg->message("ECP loaded");
}

void speaking::main_task_algorithm(void)
{
	for(;;)
	{
		sr_ecp_msg->message("NOWA SERIA");
		sr_ecp_msg->message("Ruch");
		sr_ecp_msg->message("Zakocz - nacisnij PULSE ECP trigger");
		speak->Move();
	}
}

}
} // namespace speaker

namespace common {
namespace task {

task* return_created_ecp_task (lib::configurator &_config)
{
	return new speaker::task::speaking(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

