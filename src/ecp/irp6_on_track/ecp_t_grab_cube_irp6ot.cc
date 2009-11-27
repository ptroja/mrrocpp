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
};

void grab_cube::main_task_algorithm(void ) {


	smoothgen2->set_absolute();
	//smoothgen2->load_file_with_path("/net/koleszko/mnt/mrroc/trj/smooth2test2.trj");
	smoothgen2->load_coordinates(lib::JOINT,0,-0.013,-1.442,-0.275,0.01,4.686,-0.070,0.090,true);
	smoothgen2->Move();
	smoothgen2->reset();

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


