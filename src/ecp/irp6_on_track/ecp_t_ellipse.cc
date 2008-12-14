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
    sr_ecp_msg->message("ECP loaded");
};


void ecp_task_ellipse::main_task_algorithm(void)
{
    sr_ecp_msg->message("ECP Elipsorysowacz  - wcisnij start");
    ecp_wait_for_start();

	//Polosie elipsy
	double a,b;
	
	a = read_double((char*)"a",0,MAX_MAJOR);	
	b = read_double((char*)"b",0,MAX_MINOR);
    eg = new ecp_ellipse_generator(*this,a,b,100);	
    eg->Move();
    ecp_termination_notice();
    ecp_wait_for_stop();
};

double ecp_task_ellipse::read_double(char* name,double min,double max)
{
	double value;
	int cnt = 0;

	if(min > max)
	{
		min += max;
		max = min - max;
		min = min - max;
	}
	
	//bufor pomocniczy
	char tmp[666];

	while(true)
	{
		if(cnt > 0)
		{
			sprintf(tmp,"BLAD! Podaj '%s' [%.3f;%.3f]",name,min,max);
		}
		else
		{
			sprintf(tmp,"Podaj '%s' [%.3f;%.3f]",name,min,max);
		}
		
		value = input_double(tmp);
		if(value >= min && value <= max)
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
