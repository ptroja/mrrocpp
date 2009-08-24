/*
 * ecp_vis_ib_eih_follower_irp6ot.cc
 *
 *  Created on: Dec 9, 2008
 *      Author: rtulwin
 */

#include "ecp/irp6_on_track/ecp_vis_ib_eih_follower_irp6ot.h"
#include <math.h>


namespace mrrocpp {
namespace ecp {
namespace irp6ot {

ecp_vis_ib_eih_follower_irp6ot::ecp_vis_ib_eih_follower_irp6ot(common::task::task& _ecp_task) :
	common::ecp_visual_servo(_ecp_task) {
	//retrieve_parameters();
	v_max[2] = 0.01;
	a_max[2] = 0.001;
	z_stop = false;
	s_z = 0.1;
}

ecp_vis_ib_eih_follower_irp6ot::~ecp_vis_ib_eih_follower_irp6ot() {

}

bool ecp_vis_ib_eih_follower_irp6ot::first_step() {
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	the_robot->EDP_data.instruction_type = lib::GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->EDP_data.motion_type = lib::ABSOLUTE;
	vsp_fradia->to_vsp.haar_detect_mode = lib::WITHOUT_ROTATION;
	first_move =  true;
	//z_counter = 0;

	ecp_t.sr_ecp_msg->message("PIERWSZY");

	return true;
}

bool ecp_vis_ib_eih_follower_irp6ot::next_step_without_constraints() {

	the_robot->EDP_data.instruction_type = lib::SET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.set_type = ARM_DV;
	the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->EDP_data.get_arm_type = lib::JOINT;
	the_robot->EDP_data.motion_type = lib::ABSOLUTE;
	the_robot->EDP_data.next_interpolation_type = lib::MIM;
	the_robot->EDP_data.motion_steps = MOTION_STEPS;
	the_robot->EDP_data.value_in_step_no = MOTION_STEPS - 1;

	double t = MOTION_STEPS * 2;

	if (first_move == true) {

		memcpy(next_position,
	 			the_robot->EDP_data.current_XYZ_AA_arm_coordinates, 6
						* sizeof(double));
		next_position[6] = the_robot->EDP_data.current_gripper_coordinate;

		v[0] = 0;
		v[1] = 0;
		v[2] = 0;
		//ruch w z
		z_start = next_position[2];

		s_acc = (v_max[2] * v_max[2]) / (2 * a_max[2]);

		if ((2 * s_acc) > s_z) {
			printf("redukcja predkosci z\n");
			v_max[2] = sqrt(s_z * a_max[2]);
			s_acc = (v_max[2] * v_max[2]) / (2 * a_max[2]);
		}

		//double z_t = 2 * v_max[2]/a_max[2] + (s_z - 2 * s_acc)/v_max[2];

	    /*if(ceil(z_t/t)*t != z_t)//zaokraglenie czasu do wielokrotnosci trwania makrokroku
	    {
	        z_t = ceil(z_t/t);
	        z_t = z_t*t;
	    }*/

		first_move = false;
	}
	//ruch w z
	if (v[2] < v_max[2] && (s_z - s_acc) > fabs(next_position[2] - z_start)) {//przyspieszanie
		s[2] = (a_max[2] * t * t)/2 + (v[2] * t);
		v[2] += a_max[2] * t;
	} else if (v[2] > 0 && (s_z - s_acc) <= fabs(next_position[2] - z_start)) {//hamowanie
		s[2] = (a_max[2] * t * t)/2 + (v[2] * t);
		v[2] -= a_max[2] * t;
		if (v[2] < 0) {
			z_stop = true;
		}
	} else {//jednostajny
		s[2] = v_max[2] * t;
	}

	if (z_stop == true) {
		next_position[2] = z_start - s_z;
	} else if (fabs(next_position[2] - z_start) < s_z) {
		next_position[2] += s[2];
	}
	/*if (z_counter < 5) {
		next_position[2] -= 0.005;
		z_counter++;
	}*/

	alpha = the_robot->EDP_data.current_joint_arm_coordinates[1]- the_robot->EDP_data.current_joint_arm_coordinates[6];
	//Uchyb wyrazony w pikselach.
	//double ux = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.x;
	//double uy = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.y;

	memcpy(the_robot->EDP_data.next_XYZ_AA_arm_coordinates, next_position,
			6 * sizeof(double));

	the_robot->EDP_data.next_gripper_coordinate = next_position[6];

	return true;
}

void ecp_vis_ib_eih_follower_irp6ot::entertain_constraints() {

}

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


