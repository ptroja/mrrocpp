/*
 * ecp_vis_ib_eih_object_tracker_irp6ot.cc
 *
 *  Created on: Sept 29, 2009
 *      Author: rtulwin
 */

#include "ecp/irp6_on_track/ecp_vis_ib_eih_object_tracker_irp6ot.h"
#include <math.h>


namespace mrrocpp {
namespace ecp {
namespace irp6ot {

ecp_vis_ib_eih_object_tracker_irp6ot::ecp_vis_ib_eih_object_tracker_irp6ot(common::task::task& _ecp_task) :
	common::ecp_visual_servo(_ecp_task) {

	for (int i = 0; i <= 6; i++) {//ustawianie next_position dla wszystkich osi na 0
		next_position[i] = 0;
	}

	t = MOTION_STEPS * STEP;


	v_max[1] = v_max[0] = 0.012;//TODO to wszystko bedzie w metodzie wczytujacej z pliku
	a_max[1] = a_max[0] = 0.01;
	v_max[2] = 0.03;
	a_max[2] = 0.01;
	v_stop[0] = v_stop[1] = 0.0005;
	v_min[0] = 	v_min[1] = 0.0015;
	//s_z = 0.35;

	axes_num = 2;//TODO wczytywanie z pliku konfiguracyjnego
}

bool ecp_vis_ib_eih_object_tracker_irp6ot::first_step() {
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

	//z_s = 0;
	//z_stop = false;

	v_max[1] = v_max[0] = 0.012; //TODO tutaj powinna być metoda wczytująca wszystko z pliku

	//pierwsze ustawienie flag dotarcia i kierunku
	for (int i = 0; i < axes_num; i++) {
		reached[i] = false;
		dir[i] = 1;
	}

	//ecp_t.sr_ecp_msg->message("PIERWSZY");

	return true;
}

bool ecp_vis_ib_eih_object_tracker_irp6ot::next_step_without_constraints() {

	if (first_move == true) {
		the_robot->EDP_data.instruction_type = lib::SET_GET;//TODO sprawdzic czy to moze byc robione tylko raz
		//printf("poczatek sledzenia\n");
		//flushall();
		/*memcpy(next_position,
	 			the_robot->EDP_data.current_XYZ_AA_arm_coordinates, 6
						* sizeof(double));*/
		//next_position[6] = the_robot->EDP_data.current_gripper_coordinate;

		v[0] = 0;
		v[1] = 0;
		v[2] = 0;

		//ruch w z
		//z_start = next_position[2];

		//s_acc = (v_max[2] * v_max[2]) / (2 * a_max[2]);

		/*if ((2 * s_acc) > s_z) {
			//TODO przetestowac
			v_max[2] = sqrt(s_z * a_max[2]);
			s_acc = (v_max[2] * v_max[2]) / (2 * a_max[2]);
		}*/

		//double z_t = 2 * v_max[2]/a_max[2] + (s_z - 2 * s_acc)/v_max[2];

	   /* if(ceil(z_t/t)*t != z_t)//zaokraglenie czasu do wielokrotnosci trwania makrokroku (dla ruchu w z)
	    {
	        z_t = ceil(z_t/t);
	        z_t = z_t*t;
	        reduce_velocity(a_max[2], z_t, s_z, 2); //nadpisanie v_max[2]
	        s_acc = (v_max[2] * v_max[2]) / (2 * a_max[2]);
	    }*/
	    //ruch w z (koniec)

		first_move = false;
	}
	//ruch w z
	/*if (v[2] < v_max[2] && (s_z - s_acc) > z_s) {//przyspieszanie
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
	}*/
	//ruch w z (koniec)

	//alpha = the_robot->EDP_data.current_joint_arm_coordinates[1]- the_robot->EDP_data.current_joint_arm_coordinates[6];
	//Uchyb wyrazony w pikselach.

	lib::VSP_REPORT vsp_report = vsp_fradia->from_vsp.vsp_report;
	if (vsp_report == lib::VSP_REPLY_OK) {

		if (fabs(u[0]) < 5 && fabs(u[1]) < 5 && v[0] <= v_stop[0] & v[1] <= v_stop[1]) {//TODO przemyśleć, poprawić
			//if (z_stop) {
				//printf("koniec sledzenia\n");
				//flushall();
				return false;
			//}
		}

		u[0] = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.x;//TODO zapytać czy da sie to zrobic w petli
		u[1] = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.y;
		u[2] = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.z;

		for (int i = 0; axes_num < 2; i++) {

			tracking = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.tracking;

			//funkcja zmniejszajaca predkosc w x i y w zaleznosci od odleglosci od przebytej drogi w z
			/*double u_param;
			if (fabs(u[i]) == 0 || z_stop) {
				u_param = 10000;//dowolna relatywnie duza liczba daje zredukowanie predkosci do minimalnej przy zakonczonym z
			} else {
				u_param = 1/fabs(u[i]);
			}*/
			//im mniejsza liczba przy wspolczynniku tym wieksze znaczenie wspolczynnika
			//v_max[i] = v_max[i] - ((z_s * z_s) * 0.0005 * (u_param * 0.01));//TODO ta funkcje trzeba przerobic... (delikatnie mowiac)
			if (v_max[i] < v_min[i]) {//potrzebne jesli predkosc jest w jakis sposob redukowana
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
			6 * sizeof(double)); //zapisanie pozycji w angle axes

	the_robot->EDP_data.next_gripper_coordinate = next_position[6]; //zapisanie pozycji grippera

	return true;
}

/*void ecp_vis_ib_eih_object_tracker_irp6ot::reduce_velocity(double a, double t, double s, int i) {

	double delta;//delta w rownaniu kwadratowym

	delta = (2 * a * t) *
			(2 * a * t) +
			8 * ( -2 * a * s);

	v_max[i] = (-(2 * a * t) + sqrt(delta)) / (-4);
}*/

void ecp_vis_ib_eih_object_tracker_irp6ot::entertain_constraints() {

}

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
