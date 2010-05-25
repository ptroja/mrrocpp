#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp_t_grab_cube_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

//Constructors
grab_cube::grab_cube(lib::configurator &_config) :
	task(_config) {
	ecp_m_robot = new irp6ot_m::robot(*this);

	smoothgen2 = new common::generator::smooth(*this, true);
	befgen = new common::generator::bias_edp_force(*this);
	gagen = new common::generator::tff_gripper_approach(*this, 8); //gripper approach constructor (task&, no_of_steps)
	tracker = new generator::ecp_vis_ib_eih_object_tracker_irp6ot(*this);
	turner = new generator::ecp_vis_ib_eih_wrist_turner_irp6ot(*this);

	sensor_m[lib::SENSOR_CVFRADIA] = new ecp_mp::sensor::cvfradia(
			lib::SENSOR_CVFRADIA, "[vsp_cvfradia]", *this,
			sizeof(lib::sensor_image_t::sensor_union_t::fradia_t));
	sensor_m[lib::SENSOR_CVFRADIA]->configure_sensor();
	tracker->sensor_m = sensor_m;
	turner->sensor_m = sensor_m;
}
;

void grab_cube::main_task_algorithm(void) {

	smoothgen2->set_absolute();

	smoothgen2->load_coordinates(lib::ECP_JOINT, 0, -0.013, -1.442, -0.275,
			0.01, 4.670, -0.070, 0.090, true);//grab cube from the track (desk)
	//smoothgen2->load_coordinates(lib::ECP_JOINT,0,0,-1.57,0,1.56,1.571,-1.570,0.090,true);//grab cube from the operator
	smoothgen2->Move();
	smoothgen2->reset();

	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	vsp_fradia->get_reading();
	while (vsp_fradia->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED) {
		vsp_fradia->get_reading();
	}

	tracker->Move();

	/*double v[MAX_SERVOS_NR]={0.20, 0.20, 0.01, 0.20, 0.20, 0.20, 0.20, 0.20};
	 double a[MAX_SERVOS_NR]={0.15, 0.15, 0.5, 0.15, 0.15, 0.15, 0.15, 0.15};

	 smoothgen2->set_relative();
	 smoothgen2->load_coordinates(lib::ECP_JOINT,0,0,0,0,0,0,0,-0.017,true);
	 smoothgen2->Move();
	 smoothgen2->reset();*/
	/*
	 smoothgen2->set_relative();
	 smoothgen2->load_coordinates(lib::XYZ_ANGLE_AXIS,v,a,0.008,0,0,0,0,0,0,0,true);
	 smoothgen2->Move();
	 smoothgen2->reset();
	 */

	//vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	//vsp_fradia->get_reading();
	//while(vsp_fradia->from_vsp.vsp_report == lib::VSP_SENSOR_NOT_CONFIGURED){
	//	vsp_fradia->get_reading();
	//}

	//turner->Move();

	//befgen->Move();

	//gagen->configure(0.01,1500);
	//gagen->Move();
	/*
	 smoothgen2->set_relative();
	 smoothgen2->load_coordinates(lib::XYZ_ANGLE_AXIS,v,a,0,0,-0.001,0,0,0,0,0,true);
	 //smoothgen2->load_coordinates(lib::XYZ_ANGLE_AXIS,0,0,0,0,0,0,-0.013,0,false);	//close gripper
	 //smoothgen2->load_coordinates(lib::XYZ_ANGLE_AXIS,v,a,0,0,-0.1,0,0,0,0,0,false);
	 smoothgen2->Move();
	 smoothgen2->reset();

	 smoothgen2->set_relative();
	 smoothgen2->load_coordinates(lib::JOINT,0,0,0,0,0,0,0,-0.013,true);
	 smoothgen2->Move();
	 smoothgen2->reset();

	 smoothgen2->set_relative();
	 smoothgen2->load_coordinates(lib::XYZ_ANGLE_AXIS,v,a,0,0,-0.1,0,0,0,0,0,true);
	 smoothgen2->Move();
	 smoothgen2->reset();
	 */
	ecp_termination_notice();
}
;

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6ot_m::task::grab_cube(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


