#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/common/ecp_t_kcz_force.h"

#include "ecp_mp/ecp_mp_s_pcbird.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
kcz_force::kcz_force(lib::configurator &_config): task(_config)
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

	//delay(20000);
	nose_run = new common::generator::pcbird_nose_run(*this, 8);
	nose_run->configure_pulse_check (true);
	nose_run->sensor_m = sensor_m;

	sr_ecp_msg->message("ECP loaded kcz_force");
};

void kcz_force::main_task_algorithm(void ) {
	//ecp_m_robot = new ecp_irp6_on_track_robot(*this);
	//smoothgen2 = new ecp_smooth2_generator(*this, true);
	//sr_ecp_msg->message("ECP loaded smooth2_test");

	sr_ecp_msg->message("ECP kcz_force ready");

	char buffer[100];

	for(int i=0; i<10; i++){
		nose_run->Move();
		float tempx = sensor_m[lib::SENSOR_PCBIRD]->image.sensor_union.pcbird.x;
		float tempy = sensor_m[lib::SENSOR_PCBIRD]->image.sensor_union.pcbird.y;
		float tempz = sensor_m[lib::SENSOR_PCBIRD]->image.sensor_union.pcbird.z;
		float temp00 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[0][0];
		float temp01 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[0][1];
		float temp02 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[0][2];
		float temp03 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[0][3];
		float temp10 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[1][0];
		float temp11 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[1][1];
		float temp12 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[1][2];
		float temp13 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[1][3];
		float temp20 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[2][0];
		float temp21 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[2][1];
		float temp22 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[2][2];
		float temp23 = ecp_m_robot->reply_package.arm.pf_def.arm_frame[2][3];
		sr_ecp_msg->message("Sensor:");
		sprintf(buffer, "%f6.3 %f6.3 %f6.3", tempx, tempy, tempz);
		sr_ecp_msg->message(buffer);
		sr_ecp_msg->message("Robot:");
		sprintf(buffer, "%f6.3 %f6.3 %f6.3 %f6.3", temp00, temp01, temp02, temp03);
		sr_ecp_msg->message(buffer);
		sprintf(buffer, "%f6.3 %f6.3 %f6.3 %f6.3", temp10, temp11, temp12, temp13);
		sr_ecp_msg->message(buffer);
		sprintf(buffer, "%f6.3 %f6.3 %f6.3 %f6.3", temp20, temp21, temp22, temp23);
		sr_ecp_msg->message(buffer);
	}


	//printf("wielkosc listy: %d\n", smoothgen2->pose_list_length());
	//fflush();

	ecp_termination_notice();
};

}
} // namespace common

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new common::task::kcz_force(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


