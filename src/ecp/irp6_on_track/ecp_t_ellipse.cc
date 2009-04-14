#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include <fstream>

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_s_wiimote.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_t_ellipse.h"
#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

ellipse::ellipse(lib::configurator &_config) : base(_config) {};

void ellipse::task_initialization(void)
{
	ecp_m_robot = new ecp_irp6_on_track_robot (*this);
    sr_ecp_msg->message("ECP loaded");
    
	//create Wii-mote virtual sensor object
	sensor_m[lib::SENSOR_WIIMOTE] = new ecp_mp::sensor::wiimote(lib::SENSOR_WIIMOTE, "[vsp_wiimote]", *this, sizeof(lib::sensor_image_t::sensor_union_t::wiimote_t));
	//configure the sensor
	sensor_m[lib::SENSOR_WIIMOTE]->configure_sensor();
}

void ellipse::main_task_algorithm(void)
{
 	//Polosie elipsy
	double a,b;
	double* firstPosition;
	
	a = read_double((char*)"a",0,MAX_MAJOR);
	b = read_double((char*)"b",0,MAX_MINOR);
    sg = new common::generator::smooth(*this,true);
    eg = new generator::ellipse(*this,a,b,100);
    firstPosition = eg->getFirstPosition();
    
	sg->reset();
	sg->load_coordinates(lib::XYZ_EULER_ZYZ, firstPosition[0],firstPosition[1],firstPosition[2],firstPosition[3],firstPosition[4],firstPosition[5],firstPosition[6],firstPosition[7]);
	sg->Move();
	
    eg->sensor_m[lib::SENSOR_WIIMOTE] = sensor_m[lib::SENSOR_WIIMOTE];
    eg->Move();
    ecp_termination_notice();
}

double ellipse::read_double(char* name,double min,double max)
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

}
} // namespace irp6ot

namespace common {
namespace task {

base* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::ellipse(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


