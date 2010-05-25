/*
 * generator/ecp_g_vis_ib_eih_follower_irp6ot.cc
 *
 *  Created on: Dec 9, 2008
 *      Author: rtulwin
 */

#include "ecp_g_vis_ib_eih_follower_irp6ot.h"
#include <math.h>

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

ecp_vis_ib_eih_follower_irp6ot::ecp_vis_ib_eih_follower_irp6ot(
		common::task::task& _ecp_task) :
	common::generator::ecp_visual_servo(_ecp_task) {
	//v_max[1] = v_max[0] = 0.020;
	//a_max[1] = a_max[0] = 0.025;
	v_max[1] = v_max[0] = 0.02;
	a_max[1] = a_max[0] = 0.09;
	v_max[2] = 0.03;
	a_max[2] = 0.02;
	v_stop[0] = v_stop[1] = a_max[0] * MOTION_STEPS * 0.002;
	v_min[0] = v_min[1] = 0.0035;
	s_z = 0.35;
}

bool ecp_vis_ib_eih_follower_irp6ot::first_step() {

	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 2;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i]
				= lib::UNGUARDED_MOTION;
	}

	vsp_fradia->to_vsp.haar_detect_mode = lib::WITHOUT_ROTATION;
	first_move = true;
	z_s = 0;
	z_stop = false;
	reached[0] = false;
	reached[1] = false;
	dir[0] = 1;
	dir[1] = 1;
	v_max[1] = v_max[0] = 0.02;

	ecp_t.sr_ecp_msg->message("PIERWSZY");

	return true;

}

bool ecp_vis_ib_eih_follower_irp6ot::next_step_without_constraints() {

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	double t = MOTION_STEPS * 0.002;//TODO 0.002 trzeba zamienic na STEP z odpowiedniej biblioteki

	if (first_move == true) {

		//printf("poczatek sledzenia\n");
		//flushall();
		memcpy(next_position,
				the_robot->reply_package.arm.pf_def.arm_coordinates, 6
						* sizeof(double));

		//next_position[6] = the_robot->reply_package.arm.pf_def.gripper_coordinate;

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

		double z_t = 2 * v_max[2] / a_max[2] + (s_z - 2 * s_acc) / v_max[2];

		if (ceil(z_t / t) * t != z_t)//zaokraglenie czasu do wielokrotnosci trwania makrokroku
		{
			z_t = ceil(z_t / t);
			z_t = z_t * t;
			reduce_velocity(a_max[2], z_t, s_z, 2); //nadpisanie v_max[2]
			s_acc = (v_max[2] * v_max[2]) / (2 * a_max[2]);
		}
		//ruch w z (koniec)

		first_move = false;
	}
	//ruch w z
	if (v[2] < v_max[2] && (s_z - s_acc) > z_s) {//przyspieszanie
		s[2] = (a_max[2] * t * t) / 2 + (v[2] * t);
		v[2] += a_max[2] * t;
	} else if (v[2] >= 0 && (s_z - s_acc) <= z_s) {//hamowanie
		s[2] = (a_max[2] * t * t) / 2 + (v[2] * t);
		v[2] -= a_max[2] * t;
		if (v[2] <= 0) {
			z_stop = true;
		}
	} else {//jednostajny
		s[2] = v[2] * t;
	}

	if (z_stop == false) {
		z_s += s[2];
		next_position[2] = s[2];
		//next_position[2] = 0;
	} else {
		next_position[2] = 0;
	}
	//ruch w z (koniec)

	//alpha = the_robot->reply_package.arm.pf_def.arm_coordinates[1]- the_robot->reply_package.arm.pf_def.arm_coordinates[6];
	//Uchyb wyrazony w pikselach.
	u[0] = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.x - 20;
	u[1] = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.y;
	bool tracking =
			vsp_fradia->from_vsp.comm_image.sensor_union.tracker.tracking;

	printf("ux: %f\t", u[0]);
	printf("uy: %f\n", u[1]);

	lib::VSP_REPORT vsp_report = vsp_fradia->from_vsp.vsp_report;
	if (vsp_report == lib::VSP_REPLY_OK) {

		if (fabs(u[0]) < 20 && fabs(u[1]) < 20 && v[0] <= v_stop[0] && v[1]
				<= v_stop[1]) {
			if (z_stop) {
				printf(
						"#################################### koniec sledzenia ##################################\n");
				flushall();
				return false;
			}
		}

		for (int i = 0; i < 2; i++) {

			//funkcja zmniejszajaca predkosc w x i y w zaleznosci od odleglosci od przebytej drogi w z
			/*double u_param;
			 if (fabs(u[i]) == 0 || z_stop) {
			 u_param = 10000;//dowolna relatywnie duza liczba daje zredukowanie predkosci do minimalnej przy zakonczonym z
			 } else {
			 u_param = 1/fabs(u[i]);
			 }
			 //im mniejsza liczba przy wspolczynniku tym wieksze znaczenie wspolczynnika
			 v_max[i] = v_max[i] - ((z_s * z_s) * 0.0005 * (u_param * 0.01));//TODO ta funkcje trzeba przerobic... (delikatnie mowiac)*/

			v_max[i] = v_max[i] * ((s_z - (z_s * 0.001)) / s_z);//regulator

			if (v_max[i] < v_min[i]) {
				v_max[i] = v_min[i];
			}

			if (fabs(u[i]) < 20) {
				reached[i] = true;
				printf(
						"==================================== reached true w osi %d\n",
						i);
			} else {
				reached[i] = false;
			}

			if (dir[i] == u[i] / fabs(u[i])) {
				change[i] = false;
			} else {
				change[i] = true;
				if (v[i] == 0) {
					if (u[i] != 0) {
						dir[i] = u[i] / fabs(u[i]);//pierwsze ustawienie kierunku
					}
				}
			}

			printf(
					"v_max: %f\t v: %f\t change: %d\t reached: %d\t tracking: %d\n",
					v_max[i], v[i], change[i], reached[i], tracking);

			if (tracking == true && (v[i] == 0 || (v[i] > 0 && v[i] < v_max[i]
					&& change[i] == false && reached[i] == false))) {//przyspieszanie
				if (v[i] == 0 && change[i] == true) {
					change[i] = false;
				}
				if ((fabs(u[i]) < 20) && (v[i] <= 0)) {
					continue;
				}
				s[i] = (a_max[i] * t * t) / 2 + (v[i] * t);
				v[i] += a_max[i] * t;
				if (v[i] >= v_max[i]) {
					v[i] = v_max[i];
				}

				printf("przysp\n"); //porownywanie double jest spieprzone wiec musi byc tak...			// ta czesc warunku sprawia ze wchodzi w jednostajny przy osiagnieciu maks speeda
			} else if (v[i] > 0 && (change[i] == true || reached[i] == true
					|| tracking == false || (v[i] - v_max[i]) > 0.0001)) { //|| v[i] > v_max[i]) && !(v[i] == v_max[i] && change[i] == false && reached[i] == false && tracking == true)) {//hamowanie
				if (v[i] > v_max[i] && (v[i] - v_max[i]) / t < a_max[i]
						&& change[i] == false && reached[i] == false
						&& tracking == true) {
					v[i] = v_max[i];
					s[i] = (((v[i] - v_max[i]) / t) * t * t) / 2 + (v[i] * t);
				} else {
					v[i] -= a_max[i] * t;
					s[i] = (a_max[i] * t * t) / 2 + (v[i] * t);
				}

				if (v[i] < 0) {
					//u[0] = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.x-20;
					//u[1] = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.y;
					//bool tracking = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.tracking;
					v[i] = 0;
					s[i] = 0;
				}
				printf("ham\n");

			} else { //jednostajny
				s[i] = v[i] * t;
				printf("jednostajny\n");
			}
			next_position[i] = -(dir[i] * s[i]);
		}
	}

	/*for (int i = 0; i < 6; i++) {
	 next_position[i] = 0;
	 printf("%f\t", next_position[i]);
	 }*/
	next_position[6] = 0.0;

	homog_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(
			next_position));
	homog_matrix.get_frame_tab(
			the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

	//memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, next_position,
	//		6 * sizeof(double));


	printf("s_x: %f\t s_y %f\t s_z: %f\n", next_position[0], next_position[1],
			next_position[2]);

	printf("\n");
	for (int k = 0; k < 3; k++) {
		printf("v[%d]: %f\t", k, v[k]);
	}
	printf("\n\n");

	memcpy(next_position, the_robot->reply_package.arm.pf_def.arm_coordinates,
			6 * sizeof(double));
	for (int i = 0; i < 6; i++) {
		//	printf("%f\t", next_position[i]);
		next_position[i] = 0;
	}
	next_position[6] = 0;

	printf(
			"\n*********************************************************************************\n");
	return true;
}

void ecp_vis_ib_eih_follower_irp6ot::reduce_velocity(double a, double t,
		double s, int i) {

	double delta;//delta w rownaniu kwadratowym

	delta = (2 * a * t) * (2 * a * t) + 8 * (-2 * a * s);

	v_max[i] = (-(2 * a * t) + sqrt(delta)) / (-4);
}

void ecp_vis_ib_eih_follower_irp6ot::limit_step() {

}

} // namespace generator
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
