// ------------------------------------------------------------------------
//   ecp_t_sk.cc - sledzenie konturu wersja dla dowolnego z robotow irp6
// 
// Ostatnia modyfikacja: 2007
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/common/ecp_g_force.h"
#include "ecp_mp/ecp_mp_s_schunk.h"
#include "ecp/common/ecp_t_sk.h"



// KONSTRUKTORY
ecp_task_sk::ecp_task_sk(configurator &_config) : ecp_task(_config)
{
	nrg = NULL;
	yefg = NULL;
};

ecp_task_sk::~ecp_task_sk(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_sk::task_initialization(void) 
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (strcmp(config.section_name, "[ecp_irp6_on_track]") == 0)
		{ ecp_m_robot = new ecp_irp6_on_track_robot (*this); }
	else if (strcmp(config.section_name, "[ecp_irp6_postument]") == 0)
		{ ecp_m_robot = new ecp_irp6_postument_robot (*this); }
	
	// Powolanie czujnikow
	
	switch (ecp_m_robot->robot_name)
	{
		case ROBOT_IRP6_ON_TRACK:
			sensor_m[SENSOR_FORCE_ON_TRACK] = 
		new ecp_mp_schunk_sensor (SENSOR_FORCE_ON_TRACK, "[vsp_force_irp6ot]", *this);
		break;
		case ROBOT_IRP6_POSTUMENT:
			sensor_m[SENSOR_FORCE_POSTUMENT] = 
		new ecp_mp_schunk_sensor (SENSOR_FORCE_POSTUMENT, "[vsp_force_irp6p]", *this);
		break;
		default:
			fprintf(stderr, "%s:%d unknown robot type\n", __FILE__, __LINE__);
	}
	
	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}
	
	usleep(1000*100);
	
	nrg = new ecp_tff_nose_run_generator(*this, 8);
	nrg->sensor_m = sensor_m;
	
	nrg->configure (true, true, true, true, true, true, true);
	// tylko kierunki liniowe podatne
	// nrg->configure (true, true, true, false, false, false, true);

	yefg = new y_edge_follow_force_generator (*this, 8);
	yefg->sensor_m = sensor_m;
	
	switch (ecp_m_robot->robot_name)
	{
		case ROBOT_IRP6_ON_TRACK:
			sr_ecp_msg->message("ECP sk irp6ot loaded");
		break;
		case ROBOT_IRP6_POSTUMENT:
			sr_ecp_msg->message("ECP sk irp6p loaded"); 
		break;
		default:
			fprintf(stderr, "%s:%d unknown robot type\n", __FILE__, __LINE__);
	}
	
	// sprawdzenie dodatkowej opcji w konfiguracji dotyczacej uruchomienie zapamietywania trajektorii do pliku
	if (config.exists("save_activated"))
	{
		save_activated = (bool) config.return_int_value("save_activated");
	}
	else
	{
		save_activated = false;
	}

};


void ecp_task_sk::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP sledzenie konturu - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona
				
		for(;;) {
		
			for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
				 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
			{
				sensor_m_iterator->second->to_vsp.parameters = 1; // biasowanie czujnika
				sensor_m_iterator->second->configure_sensor();
			}
		
			sr_ecp_msg->message("NOWA SERIA");
			sr_ecp_msg->message("Wodzenie do pozycji sledzenia konturu");
			sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
			Move ( *nrg);
		
			// usuniecie listy o ile istnieje
			yefg->flush_pose_list();

			sr_ecp_msg->message("Sledzenie konturu");
			sr_ecp_msg->message("Nastepny etap - nacisnij PULSE ECP trigger");
			Move ( *yefg);
			
			if ( save_activated && operator_reaction ("Save drawing ") ) {
				sr_ecp_msg->message("Zapisywanie trajektorii");
				yefg->save_file (POSE_FORCE_TORQUE_AT_FRAME);
			}

		}
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop();
		break;
	} // koniec: for(;;) wewnetrznej

};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_sk(_config);
};
