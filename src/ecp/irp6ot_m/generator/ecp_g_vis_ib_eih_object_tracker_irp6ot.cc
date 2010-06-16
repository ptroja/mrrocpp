/**
 * \file ecp_g_vis_ib_eih_object_tracker_irp6ot.cc
 * \date 29-10-2009
 * \author rtulwin
 *
 * \brief
 */

#include <math.h>

#include "ecp_g_vis_ib_eih_object_tracker_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

ecp_vis_ib_eih_object_tracker_irp6ot::ecp_vis_ib_eih_object_tracker_irp6ot(common::task::task& _ecp_task) :
	common::generator::ecp_visual_servo(_ecp_task) {

	t = MOTION_STEPS * STEP; //ustawianie czasu makrokroku (50 milisekund)
}

bool ecp_vis_ib_eih_object_tracker_irp6ot::first_step() {

	//printf("first step\n");
	flushall();
	vsp_fradia = dynamic_cast<fradia_sensor_tracker *> (sensor_m[lib::SENSOR_CVFRADIA]);

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 2;

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	for (int i = 0; i < 7; i++) {//ustawianie next_position dla wszystkich osi (lacznie z chwytakiem) na 0
		next_position[i] = 0;
	}

	//if (read_parametres() == false) {//czytanie predkosci maksymalnych i przyspieszen z pliku konfiguracyjnego zadania, jesli sie nie powiodlo to przypisz domyslne
		v_max[1] = v_max[0] = 0.2;
		a_max[1] = a_max[0] = 0.06;
		v_max[2] = 0.08;
		a_max[2] = 0.06;
		v_stop[0] = v_stop[1] = v_stop[2] = 0.0015;
		v_min[0] = 	v_min[1] = v_min[2] = 0.0015;
		u_stop[0] = u_stop[1] = 25;
		u_stop[2] = 5000;
		u_max[0] = 300;
		u_max[1] = 250;
		u_max[2] = -60000;
	//}

	//pierwsze ustawienie flag dotarcia i kierunku
	for (int i = 0; i < MAX_AXES_NUM; i++) {
		reached[i] = false;
		dir[i] = 1;
		v[i] = 0;
		u[i] = 0;
	}

	return true;

}

bool ecp_vis_ib_eih_object_tracker_irp6ot::next_step_without_constraints() {

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;//TODO sprawdzic czy to moze byc robione tylko raz

	lib::VSP_REPORT_t vsp_report = vsp_fradia->get_report();
	if (vsp_report == lib::VSP_REPLY_OK) {

		// TODO: zapyta�� czy da sie to zrobic w petli
		// TODO: {x,y,z} mozna raczej zaimplementowa�� jako Eigen::Vector3d i wtedy uzyc operatora =.
		u[0] = vsp_fradia->get_reading_message().x;
		u[1] = vsp_fradia->get_reading_message().y;
		u[2] = vsp_fradia->get_reading_message().z;
		tracking = vsp_fradia->get_reading_message().tracking;

		printf("ux: %f\t", u[0]);
		printf("uy: %f\n", u[1]);
		printf("uz: %f\n", u[2]);
		flushall();

		if (fabs(u[0]) < u_stop[0]/5 && fabs(u[1]) < u_stop[1]/5 && fabs(u[2]) < u_stop[2]/5 &&
				v[0] <= v_stop[0] && v[1] <= v_stop[1] && v[2] <= v_stop[2] &&
				reached[0] == true && reached[1] == true && reached[2] == true && tracking == true) {//TODO przemy��le��, poprawi��
			//if (z_stop) {
				printf("koniec sledzenia\n");
				flushall();
				return false;
			//}
		}

		for (int i = 0; i < MAX_AXES_NUM; i++) {

			v_max_act[i] = v_max[i] * fabs(u[i]/u_max[i]); //kontroler p

			if (v_max_act[i] < v_min[i]) {//potrzebne jesli predkosc jest w jakis sposob redukowana
				v_max_act[i] = v_min[i];
			}

			if (v_max_act[i] > v_max[i]) { //potrzebne jesli uchyb przekroczy zalozony uchyb maksymalny
				v_max_act[i] = v_max[i];
			}

			if (fabs(u[i]) < fabs(u_stop[i])) {
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

			if (tracking == true && (v[i] == 0 || (v[i] > 0 && v[i] < v_max_act[i] && change[i] == false && reached[i] == false))) {//przyspieszanie
				if (v[i] == 0 && change[i] == true) {
					change[i] = false;
				}
				if ((fabs(u[i]) < 20) && (v[i] <= 0)) {
					continue;
				}
				s[i] = (a_max[i] * t * t)/2 + (v[i] * t);
				v[i] += a_max[i] * t;
				if (v[i] > v_max_act[i]) {
					v[i] = v_max_act[i];
				}
			} else if(v[i] > 0 && (change[i] == true || reached[i] == true || tracking == false || (v[i]-v_max_act[i]) > 0.0001)) {//hamowanie
				if (v[i] > v_max_act[i] && (v[i]-v_max_act[i])/t < a_max[i] && change[i] == false && reached[i] == false && tracking == true) {
					v[i] = v_max_act[i];
					s[i] = (((v[i]-v_max_act[i])/t) * t * t)/2 + (v[i] * t);
				} else {
					v[i] -= a_max[i] * t;
					s[i] = (a_max[i] * t * t)/2 + (v[i] * t);
				}
				if (v[i] < 0) {
					v[i] = 0;
					s[i] = 0;
				}
			} else { //jednostajny
				s[i] = v[i] * t;
			}
			next_position[i] = - (dir[i] * s[i]);
		}
	}

	homog_matrix.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(next_position));
	homog_matrix.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);

	//memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_frame, next_position,
	//		6 * sizeof(double)); //zapisanie pozycji w angle axes



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
	v_max[0] = ecp_t.config.value<double>("v_max_x");//TODO dorobi�� ��apanie wyj��tku gdy w pliku konfiguracyjnym nie ma odpowiednich zmiennych
	v_max[1] = ecp_t.config.value<double>("v_max_y");
	v_max[2] = ecp_t.config.value<double>("v_max_z");
	a_max[0] = ecp_t.config.value<double>("a_max_x");
	a_max[1] = ecp_t.config.value<double>("a_max_y");
	a_max[2] = ecp_t.config.value<double>("a_max_z");
	v_stop[0] = ecp_t.config.value<double>("v_stop_x");
	v_stop[1] = ecp_t.config.value<double>("v_stop_y");
	v_stop[2] = ecp_t.config.value<double>("v_stop_z");
	v_min[0] = ecp_t.config.value<double>("v_min_x");
	v_min[1] = ecp_t.config.value<double>("v_min_y");
	v_min[2] = ecp_t.config.value<double>("v_min_z");
	u_stop[0] = ecp_t.config.value<double>("u_stop_x");
	u_stop[1] = ecp_t.config.value<double>("u_stop_y");
	u_stop[2] = ecp_t.config.value<double>("u_stop_z");
	u_max[0] = ecp_t.config.value<double>("u_max_x");
	u_max[1] = ecp_t.config.value<double>("u_max_y");
	u_max[2] = ecp_t.config.value<double>("u_max_z");

	return true;
}

void ecp_vis_ib_eih_object_tracker_irp6ot::limit_step() {

}

} // namespace generator
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
