/*
 * generator/ecp_g_vis_ib_eih_planar_irp6ot.cc
 *
 *  Created on: Dec 9, 2008
 *      Author: pwilkows
 */

#include "ecp/irp6ot_m/generator/ecp_g_vis_ib_eih_planar_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

ecp_vis_ib_eih_planar_irp6ot::ecp_vis_ib_eih_planar_irp6ot(common::task::task& _ecp_task) :
	common::generator::ecp_visual_servo(_ecp_task)
{
	retrieve_parameters();
}

void ecp_vis_ib_eih_planar_irp6ot::retrieve_parameters()
{
	//Maksymalna wartosc  predkosci.
	v_max = ecp_t.config.value <double> ("v_max");

	//Wartosc przyspieszenia z jakim osiagana jest maksymalna predkosc.
	a = ecp_t.config.value <double> ("a");

	//Minimalna  wartosc predkosci do jakiej schodzimy przy hamowaniu.
	v_min = ecp_t.config.value <double> ("v_min");

	//Dystans wyrazony w pikselach, przy ktorym powinnismy hamowac.
	breaking_dist = ecp_t.config.value <double> ("breaking_dist");
}

bool ecp_vis_ib_eih_planar_irp6ot::first_step()
{
	/*
	 vsp_fradia = dynamic_cast<fradia_sensor_deviation *> sensor_m[lib::SENSOR_CVFRADIA];

	 the_robot->ecp_command.instruction.instruction_type = lib::GET;
	 the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	 the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
	 the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	 vsp_fradia->to_vsp = lib::WITHOUT_ROTATION;
	 t_m = MOTION_STEPS * STEP;
	 x = 0;y = 0;v = 0;s = 0;
	 breaking = false;
	 first_move =  true;

	 ecp_t.sr_ecp_msg->message("PIERWSZY");

	 //z = 0;

	 above_object = false;

	 return true;
	 */
}

bool ecp_vis_ib_eih_planar_irp6ot::next_step_without_constraints()
{

	//Odczytanie pozycji poczatkowej koncowki w reprezentacji os-kat.
	if (node_counter == 1) {
		/*
		 memcpy(next_position,
		 the_robot->reply_package.arm.pf_def.arm_coordinates, 6
		 * sizeof(double));
		 next_position[6] = the_robot->reply_package.arm.pf_def.gripper_coordinate;

		 the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
		 the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
		 the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
		 the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
		 the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
		 the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		 the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		 the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
		 the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 1;

		 memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, next_position,
		 6 * sizeof(double));
		 the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = next_position[6];
		 */
	}

	//Zadawanie ruchu wzgledem aktualnego polozenia koncowki.
	else {
		alpha = the_robot->reply_package.arm.pf_def.arm_coordinates[1]
				- the_robot->reply_package.arm.pf_def.arm_coordinates[6];
		//Uchyb wyrazony w pikselach.
		double ux = vsp_fradia->get_reading_message().x;
		double uy = vsp_fradia->get_reading_message().y;

		//Sprawdz czy jest odczyt z fradii.
		lib::VSP_REPORT_t vsp_report = vsp_fradia->get_report();
		if (vsp_report == lib::VSP_REPLY_OK) {
			//Sprawdzam czy osiagnieto odleglosc przy ktorej hamujemy.
			if (fabs(ux) < breaking_dist && fabs(uy) < breaking_dist)
				breaking = true;
			if (breaking) { //Gdy raz wkroczymy w rejon hamowania hamujemy do konca.
				if (v <= v_min)//Osiagnieto zadana predkosc przy hamowaniu.
				{
					breaking = false;
					above_object = true;
					return false;
				} else { //Wyhamuj
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
		} else {
			s = 0;
			first_move = true;
		}

		//Sprawdzamy czy nie bylo zbyt dluzej przerwy.
		if (!check_if_followed())
			return false;

		//Kierunek ruchu wzgledem ukladu chwytaka.
		double direction = atan2(-ux, -uy) + alpha;
		x = cos(direction) * s;
		y = sin(direction) * s;

		//std::cout << frame_no << " " << s << " " << x << " " << y << "<< direction << std::endl;

		next_position[0] += x;
		next_position[1] += y;
		//if (z < 10) {
		//		next_position[2] += -0.002;
		//z++;
		//}
		/*
		 the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
		 the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
		 the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
		 the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
		 the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
		 the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
		 the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
		 the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
		 the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 1;
		 memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, next_position,
		 6 * sizeof(double));
		 the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = next_position[6];
		 */
		return true;
	}

	// TODO: return value was missing
	return false;
}

bool ecp_vis_ib_eih_planar_irp6ot::check_if_followed()
{
	double frame_no = vsp_fradia->get_reading_message().frame_number;

	//Ograniczenia na ruch
	if (first_move) {
		first_move = false;
		old_frame_no = frame_no;
		holes = 0;
	} else {
		if (old_frame_no == frame_no)//ten sam obiekt
			holes++;
		else
			holes = 0;
	}
	old_frame_no = frame_no;
	//sprawdzam czy nie bylo zbyt dlugiej przerwy w wykryciach obiektu
	if (holes > 10) {
		ecp_t.sr_ecp_msg->message("Zgubiono obiekt\n");
		first_move = true;
		holes = 0;
		return false;
	} else
		return true;
}

void ecp_vis_ib_eih_planar_irp6ot::limit_step()
{
}

} // namespace generator
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


