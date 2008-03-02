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
#include "ecp/speaker/ecp_local.h"
#include "ecp/speaker/ecp_t_s.h"
#include "ecp_mp/ecp_mp_s_mic.h"


// KONSTRUKTORY
ecp_task_speaker::ecp_task_speaker(configurator &_config) : ecp_task(_config)
{
	speak = NULL;
};
ecp_task_speaker::~ecp_task_speaker(){};


// methods for ECP template to redefine in concrete classes
void ecp_task_speaker::task_initialization(void) 
{
	ecp_m_robot = new ecp_speaker_robot (*this);
	
	sensor_m[SENSOR_MIC] = 
		new ecp_mp_mic_sensor(SENSOR_MIC, "[vsp_mic]", *this);
	
	// Konfiguracja wszystkich czujnikow	
	for (std::map <SENSOR_ENUM, sensor*>::iterator sensor_m_iterator = sensor_m.begin();
		 sensor_m_iterator != sensor_m.end(); sensor_m_iterator++)
	{
		sensor_m_iterator->second->to_vsp.parameters=1; // biasowanie czujnika
		sensor_m_iterator->second->configure_sensor();
	}

	usleep(1000*100);	

	speak = new speaking_generator (*this, 8);
	speak->sensor_m = sensor_m;
		
	sr_ecp_msg->message("ECP loaded");		
};


void ecp_task_speaker::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP rcsc speaker  - wcisnij start");
	ecp_wait_for_start();
	for(;;) { // Wewnetrzna petla nieskonczona
					
		for(;;) {
			sr_ecp_msg->message("NOWA SERIA");
			sr_ecp_msg->message("Ruch");
			sr_ecp_msg->message("Zakocz - nacisnij PULSE ECP trigger");
			Move ( *speak);
		
		}
		
		// Oczekiwanie na STOP
		printf("przed wait for stop\n");
		ecp_wait_for_stop ();
		break;
	} // koniec: for(;;) wewnetrznej
	
};

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_speaker(_config);
};
