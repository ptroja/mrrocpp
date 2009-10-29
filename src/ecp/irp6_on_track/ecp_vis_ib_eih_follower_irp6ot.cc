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
	v_max[1] = v_max[0] = 0.012;
	a_max[1] = a_max[0] = 0.01;
	v_max[2] = 0.03;
	a_max[2] = 0.01;
	v_stop[0] = v_stop[1] = 0.0005;
	v_min[0] = 	v_min[1] = 0.0015;
	s_z = 0.35;
}

bool ecp_vis_ib_eih_follower_irp6ot::first_step() {
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	the_robot->EDP_data.instruction_type = lib::GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->EDP_data.motion_type = lib::RELATIVE;
	the_robot->EDP_data.set_type = ARM_DV;
	the_robot->EDP_data.set_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->EDP_data.next_interpolation_type = lib::MIM;
	the_robot->EDP_data.motion_steps = MOTION_STEPS;
	the_robot->EDP_data.value_in_step_no = MOTION_STEPS - 1;



	vsp_fradia->to_vsp.haar_detect_mode = lib::WITHOUT_ROTATION;
	first_move =  true;
	z_s = 0;
	z_stop = false;
	reached[0] = false;
	reached[1] = false;
	dir[0] = 1;
	dir[1] = 1;
	v_max[1] = v_max[0] = 0.012;

	ecp_t.sr_ecp_msg->message("PIERWSZY");

	return true;
}

bool ecp_vis_ib_eih_follower_irp6ot::next_step_without_constraints() {

	the_robot->EDP_data.instruction_type = lib::SET_GET;


	double t = MOTION_STEPS * 0.002;//TODO 0.002 trzeba zamienic na STEP z odpowiedniej biblioteki

	if (first_move == true) {

		//printf("poczatek sledzenia\n");
		//flushall();
		memcpy(next_position,
	 			the_robot->EDP_data.current_XYZ_AA_arm_coordinates, 6
						* sizeof(double));
		//next_position[6] = the_robot->EDP_data.current_gripper_coordinate;

		next_position[3] = 0;
		next_position[4] = 0;
		next_position[5] = 0;
		next_position[6] = 0;

		v[0] = 0;
		v[1] = 0;
		v[2] = 0;

		//ruch w z
		z_start = next_position[2];

		s_acc = (v_max[2] * v_max[2]) / (2 * a_max[2]);

		if ((2 * s_acc) > s_z) {
			//TODO przetestowac
			v_max[2] = sqrt(s_z * a_max[2]);
			s_acc = (v_max[2] * v_max[2]) / (2 * a_max[2]);
		}

		double z_t = 2 * v_max[2]/a_max[2] + (s_z - 2 * s_acc)/v_max[2];

	    if(ceil(z_t/t)*t != z_t)//zaokraglenie czasu do wielokrotnosci trwania makrokroku
	    {
	        z_t = ceil(z_t/t);
	        z_t = z_t*t;
	        reduce_velocity(a_max[2], z_t, s_z, 2); //nadpisanie v_max[2]
	        s_acc = (v_max[2] * v_max[2]) / (2 * a_max[2]);
	    }
	    //ruch w z (koniec)

		first_move = false;
	}
	//ruch w z
	if (v[2] < v_max[2] && (s_z - s_acc) > z_s) {//przyspieszanie
		s[2] = (a_max[2] * t * t)/2 + (v[2] * t);
		v[2] += a_max[2] * t;
	} else if (v[2] > 0 && (s_z - s_acc) <= z_s) {//hamowanie
		s[2] = (a_max[2] * t * t)/2 + (v[2] * t);
		v[2] -= a_max[2] * t;
		if (v[2] < 0) {
			z_stop = true;
		}
	} else {//jednostajny
		s[2] = v[2] * t;
	}

	if (z_stop == false) {
		z_s += s[2];
		next_position[2] = s[2];
		//next_position[2] = 0;
	}
	//ruch w z (koniec)

	//alpha = the_robot->EDP_data.current_joint_arm_coordinates[1]- the_robot->EDP_data.current_joint_arm_coordinates[6];
	//Uchyb wyrazony w pikselach.
	u[0] = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.x - 40;
	u[1] = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.y + 40;
	bool tracking = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.tracking;

	lib::VSP_REPORT vsp_report = vsp_fradia->from_vsp.vsp_report;
	if (vsp_report == lib::VSP_REPLY_OK) {

		if (fabs(u[0]) < 5 && fabs(u[1]) < 5 && v[0] <= v_stop[0] & v[1] <= v_stop[1]) {
			if (z_stop) {
				//printf("koniec sledzenia\n");
				//flushall();
				return false;
			}
		}

		for (int i = 0; i < 2; i++) {

			//funkcja zmniejszajaca predkosc w x i y w zaleznosci od odleglosci od przebytej drogi w z
			double u_param;
			if (fabs(u[i]) == 0 || z_stop) {
				u_param = 10000;//dowolna relatywnie duza liczba daje zredukowanie predkosci do minimalnej przy zakonczonym z
			} else {
				u_param = 1/fabs(u[i]);
			}
			//im mniejsza liczba przy wspolczynniku tym wieksze znaczenie wspolczynnika
			v_max[i] = v_max[i] - ((z_s * z_s) * 0.0005 * (u_param * 0.01));//TODO ta funkcje trzeba przerobic... (delikatnie mowiac)
			if (v_max[i] < v_min[i]) {
				v_max[i] = v_min[i];
			}

			if (fabs(u[i]) < 25) {
				reached[i] = true;
			} else {
				reached[i] = false;
			}

			if (dir[i] == u[i]/fabs(u[i])) {
				change[i] = false;
			} else {
				change[i] = true;
				if (v[i] == 0) {
					if (u[i] != 0) {
						dir[i] = u[i]/fabs(u[i]);//pierwsze ustawienie kierunku
					}
				}
			}

			if (tracking == true && (v[i] == 0 || (v[i] > 0 && v[i] < v_max[i] && change[i] == false && reached[i] == false))) {//przyspieszanie
				if (v[i] == 0 && change[i] == true) {
					change[i] = false;
				}
				s[i] = (a_max[i] * t * t)/2 + (v[i] * t);
				v[i] += a_max[i] * t;
			} else if(v[i] > 0 && (change[i] == true || reached[i] == true || tracking == false || v[i] > v_max[i])) {//hamowanie
				s[i] = (a_max[i] * t * t)/2 + (v[i] * t);
				v[i] -= a_max[i] * t;
				if (v[i] < 0) {
					v[i] = 0;
				}
			} else { //jednostajny
				s[i] = v[i] * t;
			}
			next_position[i] = - (dir[i] * s[i]);
		}
	}

	memcpy(the_robot->EDP_data.next_XYZ_AA_arm_coordinates, next_position,
			6 * sizeof(double));

	the_robot->EDP_data.next_gripper_coordinate = next_position[6];

	return true;
}

void ecp_vis_ib_eih_follower_irp6ot::reduce_velocity(double a, double t, double s, int i) {

	double delta;//delta w rownaniu kwadratowym

	delta = (2 * a * t) *
			(2 * a * t) +
			8 * ( -2 * a * s);

	v_max[i] = (-(2 * a * t) + sqrt(delta)) / (-4);
}

void ecp_vis_ib_eih_follower_irp6ot::entertain_constraints() {

}

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
