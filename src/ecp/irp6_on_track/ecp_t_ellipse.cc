#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_wiimote.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_ellipse.h"
#include "lib/mathtr.h"

ecp_task_ellipse::ecp_task_ellipse(configurator &_config) : ecp_task(_config) {};

void ecp_task_ellipse::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    sr_ecp_msg->message("ECP loaded");
    
	//create Wii-mote virtual sensor object
	sensor_m[SENSOR_WIIMOTE] = new ecp_mp_wiimote_sensor(SENSOR_WIIMOTE, "[vsp_wiimote]", *this, sizeof(sensor_image_t::sensor_union_t::fradia_t));
	//configure the sensor
	sensor_m[SENSOR_WIIMOTE]->configure_sensor();
}

void ecp_task_ellipse::main_task_algorithm(void)
{
 	//Polosie elipsy
	double a,b;

	a = read_double((char*)"a",0,MAX_MAJOR);
	b = read_double((char*)"b",0,MAX_MINOR);
    eg = new ecp_ellipse_generator(*this,a,b,100);
    sensor_m[SENSOR_WIIMOTE]->initiate_reading();
    eg->Move();
    ecp_termination_notice();
}

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
}

