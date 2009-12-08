#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>


#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/irp6_on_track/ecp_t_grab_cube_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//Constructors
grab_cube::grab_cube(lib::configurator &_config): task(_config)
{
	ecp_m_robot = new robot(*this);

	smoothgen2 = new common::generator::smooth2(*this, true);
	tracker = new ecp_vis_ib_eih_object_tracker_irp6ot(*this);

	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(lib::SENSOR_CVFRADIA,"[vsp_cvfradia]", *this,	sizeof(lib::sensor_image_t::sensor_union_t::fradia_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();
	tracker->sensor_m = sensor_m;
};

void grab_cube::main_task_algorithm(void ) {


	smoothgen2->set_absolute();
	//smoothgen2->load_file_with_path("/net/koleszko/mnt/mrroc/trj/smooth2test2.trj");
	//smoothgen2->load_coordinates(lib::JOINT,0,-0.013,-1.442,-0.275,0.01,4.686,-0.070,0.090,true);
	smoothgen2->load_coordinates(lib::JOINT,0,0,-1.57,0,1.56,1.571,-1.570,0.090,true);
	smoothgen2->Move();
	smoothgen2->reset();

	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	vsp_fradia->get_reading();
	while(vsp_fradia->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
		vsp_fradia->get_reading();
	}

	tracker->Move();

	ecp_termination_notice();
};

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config){
	return new irp6ot::task::grab_cube(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


