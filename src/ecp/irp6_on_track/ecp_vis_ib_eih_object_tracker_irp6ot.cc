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

	t = MOTION_STEPS * STEP; //ustawianie czasu makrokroku (50 milisekund)

	//s_z = 0.35;

	//axes_num = 2;
}

bool ecp_vis_ib_eih_object_tracker_irp6ot::first_step() {

	printf("first step\n");
	flushall();
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 1;

	for (int i=0; i<6; i++)
	{
		the_robot->EDP_data.next_behaviour[i] = lib::UNGUARDED_MOTION;
	}

	for (int i = 0; i < 7; i++) {//ustawianie next_position dla wszystkich osi (lacznie z chwytakiem) na 0
		next_position[i] = 0;
	}

	//vsp_fradia->to_vsp.haar_detect_mode = lib::WITHOUT_ROTATION;
	//first_move =  true;

	//z_s = 0;
	//z_stop = false;

	//if (read_parametres() == false) {//czytanie predkosci maksymalnych i przyspieszen z pliku konfiguracyjnego zadania, jesli sie nie powiodlo to przypisz domyslne
		v_max[1] = v_max[0] = 0.02;
		a_max[1] = a_max[0] = 0.02;
		v_max[2] = 0.03;
		a_max[2] = 0.02;
		v_stop[0] = v_stop[1] = v_stop[2] = 0.0005;
		v_min[0] = 	v_min[1] = v_min[2] = 0.0015;
	//}

	//pierwsze ustawienie flag dotarcia i kierunku
	for (int i = 0; i < MAX_AXES_NUM; i++) {
		reached[i] = false;
		dir[i] = 1;
		v[i] = 0;
	}

	//ecp_t.sr_ecp_msg->message("PIERWSZY");

	return true;
}

bool ecp_vis_ib_eih_object_tracker_irp6ot::next_step_without_constraints() {

	//if (first_move == true) {
		the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;//TODO sprawdzic czy to moze byc robione tylko raz
		//printf("poczatek sledzenia\n");
		//flushall();
		/*memcpy(next_position,
	 			the_robot->reply_package.arm.pf_def.arm_coordinates, 6
						* sizeof(double));*/
		//next_position[6] = the_robot->EDP_data.current_gripper_coordinate;

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

	//	first_move = false;
	//}
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

	//alpha = the_robot->reply_package.arm.pf_def.arm_coordinates[1]- the_robot->reply_package.arm.pf_def.arm_coordinates[6];
	//Uchyb wyrazony w pikselach.

	lib::VSP_REPORT vsp_report = vsp_fradia->from_vsp.vsp_report;
	if (vsp_report == lib::VSP_REPLY_OK) {

		u[0] = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.x;//TODO zapytać czy da sie to zrobic w petli
		u[1] = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.y;
		u[2] = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.z;
		tracking = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.tracking;

		//u[0] = 0;
		//u[1] = 0;
		//u[2] = -100;

		printf("ux: %f\t", u[0]);
		printf("uy: %f\n", u[1]);
		printf("uz: %f\n", u[2]);
		flushall();

		if (fabs(u[0]) < 5 && fabs(u[1]) < 5 && fabs(u[2]) < 5 && v[0] <= v_stop[0] && v[1] <= v_stop[1] && v[2] <= v_stop[2]) {//TODO przemyśleć, poprawić
			//if (z_stop) {
				printf("koniec sledzenia\n");
				//flushall();
				return false;
			//}
		}

		for (int i = 0; i < MAX_AXES_NUM; i++) {
			if (u[i] == 0) {
				continue;
			}

			//tracking = true;
			//tracking = vsp_fradia->from_vsp.comm_image.sensor_union.tracker.tracking;

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
				if (v[i] > v_max[i]) {
					v[i] = v_max[i];
				}
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

	memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, next_position,
			6 * sizeof(double)); //zapisanie pozycji w angle axes

	the_robot->EDP_data.next_gripper_coordinate = next_position[6]; //zapisanie pozycji grippera

	printf("s_x: %f\t s_y %f\t s_z: %f\n", next_position[0],next_position[1],next_position[2]);

	printf("\n");
	for (int k = 0; k < 3; k++) {
		printf("v[%d]: %f\t", k, v[k]);
	}
	printf("\n\n");
	printf("\n*********************************************************************************\n");

	for (int i = 0; i <= 7; i++) {//ustawianie next_position dla wszystkich osi (lacznie z chwytakiem) na 0
		next_position[i] = 0;
	}

	flushall();

	return true;
}

bool ecp_vis_ib_eih_object_tracker_irp6ot::read_parametres() {//metoda wczytujaca predkosci, przyspieszenia etc z pliku konfiguracyjnego
															//przykladowe wartosci podane sa w first stepie
	v_max[0] = ecp_t.config.return_double_value("v_max_x");//TODO dorobić łapanie wyjątku gdy w pliku konfiguracyjnym nie ma odpowiednich zmiennych
	v_max[1] = ecp_t.config.return_double_value("v_max_y");
	v_max[2] = ecp_t.config.return_double_value("v_max_z");
	a_max[0] = ecp_t.config.return_double_value("a_max_x");
	a_max[1] = ecp_t.config.return_double_value("a_max_y");
	a_max[2] = ecp_t.config.return_double_value("a_max_z");
	v_stop[0] = ecp_t.config.return_double_value("v_stop_x");
	v_stop[1] = ecp_t.config.return_double_value("v_stop_y");
	v_stop[2] = ecp_t.config.return_double_value("v_stop_z");
	v_min[0] = ecp_t.config.return_double_value("v_min_x");
	v_min[1] = ecp_t.config.return_double_value("v_min_y");
	v_min[2] = ecp_t.config.return_double_value("v_min_z");

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
