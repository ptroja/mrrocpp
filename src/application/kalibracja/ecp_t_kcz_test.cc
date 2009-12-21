#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "application/kalibracja/ecp_t_kcz_test.h"
#include "ecp_mp/sensor/ecp_mp_s_pcbird.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
kcz_test::kcz_test(lib::configurator &_config): task(_config)
{
    if (config.section_name == ECP_IRP6_ON_TRACK_SECTION)
    {
        ecp_m_robot = new irp6ot::robot (*this);
    }
    else if (config.section_name == ECP_IRP6_POSTUMENT_SECTION)
    {
        ecp_m_robot = new irp6p::robot (*this);
    }

	sensor_m[lib::SENSOR_PCBIRD] = new ecp_mp::sensor::pcbird("[vsp_pcbird]", *this);
	sensor_m[lib::SENSOR_PCBIRD]->configure_sensor();

	smoothgen2 = new common::generator::smooth(*this, true);
	smoothgen2->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP loaded kcz_test");
};

void kcz_test::main_task_algorithm(void ) {
	sr_ecp_msg->message("ECP kcz_test ready");

	smoothgen2->set_absolute();

	smoothgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, -0.289, 1.08, -0.2, 0.762*3.11, 0.648*3.11, -0.019*3.11, 0.074, 0.000, false);
	smoothgen2->Move();

	double vv[MAX_SERVOS_NR]={0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
	double aa[MAX_SERVOS_NR]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

	do {
	smoothgen2->load_coordinates(lib::ECP_XYZ_ANGLE_AXIS, vv, aa, sensor_m[lib::SENSOR_PCBIRD]->image.sensor_union.pcbird.x - 0.315, -1.0 * sensor_m[lib::SENSOR_PCBIRD]->image.sensor_union.pcbird.y + 1.216, -1.0 * sensor_m[lib::SENSOR_PCBIRD]->image.sensor_union.pcbird.z - 0.081 + 0.1, 0.762*3.11, 0.648*3.11, -0.019*3.11, 0.074, 0.000, false);
	smoothgen2->Move();
	} while(true);

	smoothgen2->reset();

	ecp_termination_notice();

};

}
} // namespace common

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new common::task::kcz_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


