#include <cstring>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include <fstream>

#include "base/lib/sr/srlib.h"
#include "application/wii_teach/sensor/ecp_mp_s_wiimote.h"

#include "base/ecp/ecp_task.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "application/wii_ellipse/ecp_t_wii_ellipse.h"
#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

wii_ellipse::wii_ellipse(lib::configurator &_config) :
	task(_config) {
	ecp_m_robot = new irp6ot_m::robot(*this);

	//create Wii-mote virtual sensor object
	sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE] = new ecp_mp::sensor::wiimote(ecp_mp::sensor::SENSOR_WIIMOTE, "[vsp_wiimote]", *this->sr_ecp_msg, this->config);
	//configure the sensor
	sensor_m[ecp_mp::sensor::SENSOR_WIIMOTE]->configure_sensor();
}

void wii_ellipse::main_task_algorithm(void) {
	/*
	 //Polosie elipsy
	 double a,b;
	 double* firstPosition;

	 a = read_double((char*)"a",0,MAX_MAJOR);
	 b = read_double((char*)"b",0,MAX_MINOR);
	 sg = new common::generator::smooth(*this,true);
	 eg = new generator::wii_ellipse(*this,a,b,100);
	 firstPosition = eg->getFirstPosition();

	 sg->reset();
	 sg->load_coordinates(lib::XYZ_EULER_ZYZ, firstPosition[0],firstPosition[1],firstPosition[2],firstPosition[3],firstPosition[4],firstPosition[5],firstPosition[6],firstPosition[7]);
	 sg->Move();

	 eg->sensor_m[lib::SENSOR_WIIMOTE] = sensor_m[lib::SENSOR_WIIMOTE];
	 eg->Move();
	 ecp_termination_notice();
	 */
}

double wii_ellipse::read_double(char* name, double min, double max) {
	double value;
	int cnt = 0;

	if (min > max) {
		min += max;
		max = min - max;
		min = min - max;
	}

	//bufor pomocniczy
	char tmp[666];

	while (true) {
		if (cnt > 0) {
			sprintf(tmp, "BLAD! Podaj '%s' [%.3f;%.3f]", name, min, max);
		} else {
			sprintf(tmp, "Podaj '%s' [%.3f;%.3f]", name, min, max);
		}

		value = input_double(tmp);
		if (value >= min && value <= max) {
			return value;
		}

		++cnt;
	}
}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6ot_m::task::wii_ellipse(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


