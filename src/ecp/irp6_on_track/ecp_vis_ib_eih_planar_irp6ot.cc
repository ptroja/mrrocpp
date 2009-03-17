/*
 * ecp_vis_ib_eih_planar_irp6ot.cc
 *
 *  Created on: Dec 9, 2008
 *      Author: pwilkows
 */

#include "ecp/irp6_on_track/ecp_vis_ib_eih_planar_irp6ot.h"

ecp_vis_ib_eih_planar_irp6ot::ecp_vis_ib_eih_planar_irp6ot(ecp_task& _ecp_task) :
	ecp_visual_servo(_ecp_task) {

	v_max = 0.02;
	//Wartosc przyspieszenia z jakim osiagana jest maksymalna predkosc.
	a = 0.04;
	//Minimalna  wartosc predkosci do jakiej schodzimy przy hamowaniu.
	v_min = 0.002;
	//Predkosc chwilowa.

	//Dystans wyrazony w pikselach, przy ktorym nastepuje hamowanie.
	breaking_dist = 7;
}

ecp_vis_ib_eih_planar_irp6ot::~ecp_vis_ib_eih_planar_irp6ot() {

}

void ecp_vis_ib_eih_planar_irp6ot::retrieve_parameters() {
	//Maksymalna wartosc  predkosci.
	v_max = ecp_t.config.return_double_value("v_max");

	//Wartosc przyspieszenia z jakim osiagana jest maksymalna predkosc.
	a = ecp_t.config.return_double_value("a");

	//Minimalna  wartosc predkosci do jakiej schodzimy przy hamowaniu.
	v_min = ecp_t.config.return_double_value("v_min");

	//Dystans wyrazony w pikselach, przy ktorym powinnismy hamowac.
	breaking_dist = ecp_t.config.return_double_value("breaking_dist");

}

bool ecp_vis_ib_eih_planar_irp6ot::first_step() {

	vsp_fradia = sensor_m[SENSOR_CVFRADIA];

	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.get_arm_type = XYZ_ANGLE_AXIS;
	the_robot->EDP_data.motion_type = ABSOLUTE;
	the_robot->EDP_data.next_interpolation_type = MIM;
	the_robot->EDP_data.motion_steps = MOTION_STEPS;
	the_robot->EDP_data.value_in_step_no = MOTION_STEPS - 1;

	t_m = MOTION_STEPS * STEP;

	x = 0;
	y = 0;
	v = 0;
	s = 0;
	breaking = false;

	//    for(int i=0;i<8;i++)
	//    	std::cout<<next_position[i]<<std::endl;
	return true;
}

bool ecp_vis_ib_eih_planar_irp6ot::next_step_without_constraints() {
	//Odczytanie pozycji poczatkowej koncowki w reprezentacji os-kat.
	if (node_counter == 1) {

		memcpy(next_position,
	 			the_robot->EDP_data.current_XYZ_AA_arm_coordinates, 6
						* sizeof(double));
		next_position[6] = the_robot->EDP_data.current_gripper_coordinate;

		the_robot->EDP_data.instruction_type = SET_GET;
		the_robot->EDP_data.get_type = ARM_DV;
		the_robot->EDP_data.set_type = ARM_DV;
		the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
		the_robot->EDP_data.get_arm_type = JOINT;
		the_robot->EDP_data.motion_type = ABSOLUTE;
		the_robot->EDP_data.next_interpolation_type = MIM;
		the_robot->EDP_data.motion_steps = MOTION_STEPS;
		the_robot->EDP_data.value_in_step_no = MOTION_STEPS - 1;

		memcpy(the_robot->EDP_data.next_XYZ_AA_arm_coordinates, next_position,
				6 * sizeof(double));
		the_robot->EDP_data.next_gripper_coordinate = next_position[6];

	}

	//Odczytanie orientaci koncowki, wzgledem ukladu bazowego.
	else {

		std::cout << "alpha1: " << the_robot->EDP_data.current_joint_arm_coordinates[1] <<"\n"<< std::endl;

		std::cout << "alpha2: " << the_robot->EDP_data.current_joint_arm_coordinates[6] <<"\n"<< std::endl;

		alpha = the_robot->EDP_data.current_joint_arm_coordinates[1]
				- the_robot->EDP_data.current_joint_arm_coordinates[6];

		std::cout << "alpha: " << alpha << std::endl;



		//Uchyb wyrazony w pikselach.
		double ux = vsp_fradia->from_vsp.comm_image.sensor_union.deviation.x;
		double uy = vsp_fradia->from_vsp.comm_image.sensor_union.deviation.y;
		double
				frame_no =
						vsp_fradia->from_vsp.comm_image.sensor_union.deviation.frame_number;

		if (frame_no != 0) {
			//Sprawdzam czy osiagnieto odleglosc przy ktorej hamujemy.
			if (fabs(ux) < breaking_dist && fabs(uy) < breaking_dist)
				breaking = true;
			if (breaking) { //Gdy raz wkroczymy w rejon hamowania hamujemy do konca.
				if (v <= v_min)//Osiagnieto zadana predkosc przy hamowaniu.
					return false;
				else { //Wyhamuj
					s = v * t_m - 0.5 * a * t_m * t_m;
					v = v - a * t_m;//Predkosc w nastepnym makrokroku.
					//Sprawdzam czy nie przekroczono predkosci mnimalnej.
					if (v < v_min)
						v = v_min;
				}
			} else if (v < v_max) { //Przyspieszanie
				s = v * t_m + 0.5 * a * t_m * t_m;
				v = v + a * t_m;//Predkosc w nastepnym makrokroku.
				//Sprawdzam czy nie przekroczono predkosci maxymalnej.
				if (v > v_max)
					v = v_max;
			} else if (v = v_max) { //Ruch jednostajny.
				//return false;
				s = v * t_m;
			}
		} else
			s = 0;

		double direction = atan2(-ux, -uy) + alpha;
		x = cos(direction) * s;
		y = sin(direction) * s;

		std::cout << frame_no << " " << s << " " << x << " " << y << " "
				<< direction << std::endl;

		next_position[0] += x;
		next_position[1] += y;

		the_robot->EDP_data.instruction_type = SET_GET;
		the_robot->EDP_data.get_type = ARM_DV;
		the_robot->EDP_data.set_type = ARM_DV;
		the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
		the_robot->EDP_data.get_arm_type = JOINT;
		the_robot->EDP_data.motion_type = ABSOLUTE;
		the_robot->EDP_data.next_interpolation_type = MIM;
		the_robot->EDP_data.motion_steps = MOTION_STEPS;
		the_robot->EDP_data.value_in_step_no = MOTION_STEPS - 1;
		memcpy(the_robot->EDP_data.next_XYZ_AA_arm_coordinates, next_position,
				6 * sizeof(double));
		the_robot->EDP_data.next_gripper_coordinate = next_position[6];

		return true;
	}
}

void ecp_vis_ib_eih_planar_irp6ot::entertain_constraints() {

}
