#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"
#include "ecp_mp/ecp_mp_s_schunk.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_ellipse.h"
#include "lib/mathtr.h"

ecp_task_ellipse::ecp_task_ellipse(configurator &_config) : ecp_task(_config) {};

void ecp_task_ellipse::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
	
    sg = new ecp_smooth_generator (*this, true, true);

    sr_ecp_msg->message("ECP loaded");
};


void ecp_task_ellipse::main_task_algorithm(void)
{
    sr_ecp_msg->message("ECP Elipsorysowacz  - wcisnij start");
    ecp_wait_for_start();

	//Polosie elipsy
	double a,b;

	a = read_axis((char*)"a",MAX_MAJOR);	
	b = read_axis((char*)"b",MAX_MINOR);
    sg->Move();
    ecp_termination_notice();
    ecp_wait_for_stop();
};

double ecp_task_ellipse::read_axis(char* name,double limit)
{
	double value;
	int cnt = 0;
	
	//bufor pomocniczy
	char tmp[666];

	while(true)
	{
		if(cnt > 0)
		{
			sprintf(tmp,"Nieprawidlowa wartosc\nPodaj wartosc polosi %s [0;%.3f]",name,limit);
		}
		else
		{
			sprintf(tmp,"Podaj wartosc polosi %s [0;%.3f]",name,limit);
		}
		
		value = input_double(tmp);
		if(value >= 0 && value <= limit)
		{
			return value;
		}
		
		++cnt;
	}	
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_ellipse(_config);
};
