#include <string.h>
#include <unistd.h>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_force.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_g_pawel.h"
#include "ecp/common/ecp_t_pawel.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
pawel::pawel(lib::configurator &_config) : task(_config)
{
    pg = NULL;
}

// methods for ECP template to redefine in concrete classes
void pawel::task_initialization(void)
{
    // the robot is choose dependendant on the section of configuration file sent as argv[4]
    if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
    {
        ecp_m_robot = new irp6ot::robot (*this);
    }
    else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
    {
        ecp_m_robot = new irp6p::robot (*this);
    }

    // Powolanie czujnikow

    sensor_m[lib::SENSOR_PAWEL] = new ecp_mp::sensor::pawel(lib::SENSOR_PAWEL, "[vsp_pawel]", *this);
    sensor_m[lib::SENSOR_PAWEL]->configure_sensor();
    //	sensor_m[SENSOR_PAWEL]->initiate_reading();

    delay(100);

    pg = new generator::pawel ( *this, 20 );
    pg->sensor_m = sensor_m;

    switch (ecp_m_robot->robot_name)
    {
    case lib::ROBOT_IRP6_ON_TRACK:
        sr_ecp_msg->message("ECP irp6ot loaded");
        break;
    case lib::ROBOT_IRP6_POSTUMENT:
        sr_ecp_msg->message("ECP irp6p loaded");
        break;
    default:
        break;
    }
}


void pawel::main_task_algorithm(void)
{
    //	sensor_m[SENSOR_PAWEL]->get_reading();
    for(;;)
    {
        pg->Move();
    }
}

task* return_created_ecp_task (lib::configurator &_config)
{
	return new pawel(_config);
}
	//	nrg = new ecp_tff_nose_run_generator(*this, 8);
	//	nrg->sensor_m = sensor_m;
	//	nrg->configure (true, true, true, true, true, true, true);  // wszystkie podatne
	//	tig = new ecp_teach_in_generator (*this);
	//	pg = new pawel_generator (*this);
	//	nrg->sensor_m = sensor_m;

	/*
		for (ecp_mp::sensors_t::iterator sensor_m_iterator = sensor_m.begin();
			 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
		{
			sensor_m_iterator->second->to_vsp.parameters = 1; // biasowanie czujnika
			sensor_m_iterator->second->configure_sensor();
		}
		*/
	//			if ( operator_reaction ("[p] Load trajectory? ") ) {
	//				ecp_load_file_from_ui (*tig);
	//			}
	//	sr_ecp_msg->message("[p] ECP aby zakonczyc wcisnij STOP");

	//			Move ( *nrg);
	//	Move ( *tig);

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
