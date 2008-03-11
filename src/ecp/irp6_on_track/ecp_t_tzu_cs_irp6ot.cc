#include <iostream.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"	

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/common/ecp_g_tzu_cs.h"
#include "ecp/irp6_on_track/ecp_t_tzu_cs_irp6ot.h"

/** konstruktor **/
ecp_task_tzu_cs_irp6ot::ecp_task_tzu_cs_irp6ot(configurator &_config) : ecp_task(_config)
{
};

/** destruktor **/
ecp_task_tzu_cs_irp6ot::~ecp_task_tzu_cs_irp6ot()
{
};


// methods for ECP template to redefine in concrete classes
void ecp_task_tzu_cs_irp6ot::task_initialization(void) 
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	
	sr_ecp_msg->message("ECP loaded");
};


void ecp_task_tzu_cs_irp6ot::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP cs irp6ot  - pushj start in tzu");
	ecp_wait_for_start();
	
	/* stworzenie generatora ruchu */
	// sprawdzic czy drugi argument jest rzeczywiscie potrzebny
	tzu_simple_generator sg(*this, 8);
	// sprawdzic zmienna sensor_m
	sg.sensor_m = sensor_m;
	
	while(true)			
	{ 
		// sprawdzic co robi metoda move znajdujaca sie pewnie w ecp_task
sg.Move();
		cout<<"wait for stop\n"<<endl;
		ecp_wait_for_stop();
		break;
	}
	cout<<"end\n"<<endl;
};

// sprawdzic co robi ta metoda, gdzie, w jakich przypadkach jest uzywana
ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_tzu_cs_irp6ot(_config);
};

