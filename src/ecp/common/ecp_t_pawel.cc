#include <string.h>
#include <unistd.h>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_force.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_g_pawel.h"
#include "ecp/common/ecp_t_pawel.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
ecp_task_pawel::ecp_task_pawel(configurator &_config) : ecp_task(_config)
{
    pg = NULL;
}

// methods for ECP template to redefine in concrete classes
void ecp_task_pawel::task_initialization(void)
{
    // the robot is choose dependendant on the section of configuration file sent as argv[4]
    if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
    {
        ecp_m_robot = new irp6ot::ecp_irp6_on_track_robot (*this);
    }
    else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
    {
        ecp_m_robot = new irp6p::ecp_irp6_postument_robot (*this);
    }

    // Powolanie czujnikow

    sensor_m[SENSOR_PAWEL] = new ecp_mp::sensor::pawel(SENSOR_PAWEL, "[vsp_pawel]", *this);
    sensor_m[SENSOR_PAWEL]->configure_sensor();
    //	sensor_m[SENSOR_PAWEL]->initiate_reading();

    delay(100);

    pg = new pawel_generator ( *this, 20 );
    pg->sensor_m = sensor_m;

    switch (ecp_m_robot->robot_name)
    {
    case ROBOT_IRP6_ON_TRACK:
        sr_ecp_msg->message("ECP irp6ot loaded");
        break;
    case ROBOT_IRP6_POSTUMENT:
        sr_ecp_msg->message("ECP irp6p loaded");
        break;
    default:
        break;
    }
}


void ecp_task_pawel::main_task_algorithm(void)
{
    //	sensor_m[SENSOR_PAWEL]->get_reading();
    for(;;)
    {
        pg->Move();
    }
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_pawel(_config);
}
	//	nrg = new ecp_tff_nose_run_generator(*this, 8);
	//	nrg->sensor_m = sensor_m;
	//	nrg->configure (true, true, true, true, true, true, true);  // wszystkie podatne
	//	tig = new ecp_teach_in_generator (*this);
	//	pg = new pawel_generator (*this);
	//	nrg->sensor_m = sensor_m;

	/*
		for (map <SENSOR_ENUM, ::sensor*>::iterator sensor_m_iterator = sensor_m.begin();
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
