/*!
 * @file
 * @brief File contains  tff nose run generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <iostream>

#include "base/lib/typedefs.h"

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_tff_nose_run.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

tff_nose_run::tff_nose_run(common::task::task& _ecp_task, int step) :
		common::generator::generator(_ecp_task), pulse_check_activated(false), force_meassure(false), step_no(step)
{
	generator_name = ecp_mp::generator::ECP_GEN_TFF_NOSE_RUN;

	// domyslnie wszytkie osie podatne a pulse_check nieaktywne
	configure_behaviour(lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT, lib::CONTACT);
	configure_pulse_check(false);
	configure_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	configure_force(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	configure_reciprocal_damping(lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::FORCE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING, lib::TORQUE_RECIPROCAL_DAMPING);
	configure_inertia(lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::FORCE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA, lib::TORQUE_INERTIA);

}

void tff_nose_run::conditional_execution()
{
	ecp_mp::generator::tff_nose_run::behaviour_specification_data_type beh;

	switch ((ecp_mp::generator::tff_nose_run::communication_type) ecp_t.mp_command.ecp_next_state.variant)
        {

		case ecp_mp::generator::tff_nose_run::behaviour_specification:
			ecp_t.mp_command.ecp_next_state.sg_buf.get(beh);
			break;
		case ecp_mp::generator::tff_nose_run::no_data:
			break;
		default:
			break;
	}

	configure_behaviour(beh.behaviour[0], beh.behaviour[1], beh.behaviour[2], beh.behaviour[3], beh.behaviour[4], beh.behaviour[5]);

	Move();
}

void tff_nose_run::set_force_meassure(bool fm)
{
	force_meassure = fm;
}

void tff_nose_run::configure_pulse_check(bool pulse_check_activated_l)
{
	pulse_check_activated = pulse_check_activated_l;
}

void tff_nose_run::configure_behaviour(lib::BEHAVIOUR_SPECIFICATION x, lib::BEHAVIOUR_SPECIFICATION y, lib::BEHAVIOUR_SPECIFICATION z, lib::BEHAVIOUR_SPECIFICATION ax, lib::BEHAVIOUR_SPECIFICATION ay, lib::BEHAVIOUR_SPECIFICATION az)
{
	generator_edp_data.next_behaviour[0] = x;
	generator_edp_data.next_behaviour[1] = y;
	generator_edp_data.next_behaviour[2] = z;
	generator_edp_data.next_behaviour[3] = ax;
	generator_edp_data.next_behaviour[4] = ay;
	generator_edp_data.next_behaviour[5] = az;
}

void tff_nose_run::configure_velocity(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_velocity[0] = x;
	generator_edp_data.next_velocity[1] = y;
	generator_edp_data.next_velocity[2] = z;
	generator_edp_data.next_velocity[3] = ax;
	generator_edp_data.next_velocity[4] = ay;
	generator_edp_data.next_velocity[5] = az;
}

void tff_nose_run::configure_force(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_force_xyz_torque_xyz[0] = x;
	generator_edp_data.next_force_xyz_torque_xyz[1] = y;
	generator_edp_data.next_force_xyz_torque_xyz[2] = z;
	generator_edp_data.next_force_xyz_torque_xyz[3] = ax;
	generator_edp_data.next_force_xyz_torque_xyz[4] = ay;
	generator_edp_data.next_force_xyz_torque_xyz[5] = az;
}

void tff_nose_run::configure_reciprocal_damping(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_reciprocal_damping[0] = x;
	generator_edp_data.next_reciprocal_damping[1] = y;
	generator_edp_data.next_reciprocal_damping[2] = z;
	generator_edp_data.next_reciprocal_damping[3] = ax;
	generator_edp_data.next_reciprocal_damping[4] = ay;
	generator_edp_data.next_reciprocal_damping[5] = az;
}

void tff_nose_run::configure_inertia(double x, double y, double z, double ax, double ay, double az)
{
	generator_edp_data.next_inertia[0] = x;
	generator_edp_data.next_inertia[1] = y;
	generator_edp_data.next_inertia[2] = z;
	generator_edp_data.next_inertia[3] = ax;
	generator_edp_data.next_inertia[4] = ay;
	generator_edp_data.next_inertia[5] = az;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_nose_run::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

	//	std::cout << "tff_nose_run" << node_counter << std::endl;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	the_robot->ecp_command.robot_model.tool_frame_def.tool_frame = tool_frame;

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	//	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::TOOL_FRAME;
	the_robot->ecp_command.get_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.set_arm_type = lib::PF_VELOCITY;
//	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::RELATIVE;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = step_no;
	the_robot->ecp_command.value_in_step_no = step_no - 2;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.behaviour[i] = generator_edp_data.next_behaviour[i];
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = generator_edp_data.next_velocity[i];
		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[i] = generator_edp_data.next_force_xyz_torque_xyz[i];
		the_robot->ecp_command.arm.pf_def.reciprocal_damping[i] = generator_edp_data.next_reciprocal_damping[i];
		the_robot->ecp_command.arm.pf_def.inertia[i] = generator_edp_data.next_inertia[i];
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_nose_run::next_step()
{

	//	std::cout << "tff_nose_run" << node_counter << std::endl;
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	//std::cout << "next_step" << std::endl;

	if (pulse_check_activated && check_and_null_trigger()) { // Koniec odcinka
		//	ecp_t.set_ecp_reply (lib::TASK_TERMINATED);

		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	// wyrzucanie odczytu sil

	if (force_meassure) {
		lib::Homog_matrix current_frame_wo_offset(the_robot->reply_package.arm.pf_def.arm_frame);
		current_frame_wo_offset.remove_translation();

		lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

		std::cout << "force: " << force_torque << std::endl;
	}
	return true;

}

// metoda przeciazona bo nie chcemy rzucac wyjatku wyjscia poza zakres ruchu - UWAGA napisany szkielet skorygowac cialo funkcji

void tff_nose_run::execute_motion(void)
{
	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP

	// komunikacja wlasciwa
	the_robot->send();
	if (the_robot->reply_package.reply_type == lib::ERROR) {

		the_robot->query();
		BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(EDP_ERROR));

	}
	the_robot->query();

	if (the_robot->reply_package.reply_type == lib::ERROR) {
		switch (the_robot->reply_package.error_no.error0)
		{
			case BEYOND_UPPER_D0_LIMIT:
			case BEYOND_UPPER_THETA1_LIMIT:
			case BEYOND_UPPER_THETA2_LIMIT:
			case BEYOND_UPPER_THETA3_LIMIT:
			case BEYOND_UPPER_THETA4_LIMIT:
			case BEYOND_UPPER_THETA5_LIMIT:
			case BEYOND_UPPER_THETA6_LIMIT:
			case BEYOND_UPPER_THETA7_LIMIT:
			case BEYOND_LOWER_D0_LIMIT:
			case BEYOND_LOWER_THETA1_LIMIT:
			case BEYOND_LOWER_THETA2_LIMIT:
			case BEYOND_LOWER_THETA3_LIMIT:
			case BEYOND_LOWER_THETA4_LIMIT:
			case BEYOND_LOWER_THETA5_LIMIT:
			case BEYOND_LOWER_THETA6_LIMIT:
			case BEYOND_LOWER_THETA7_LIMIT:
				break;
			default:
				BOOST_THROW_EXCEPTION(exception::nfe_r() << lib::exception::mrrocpp_error0(EDP_ERROR));
				break;

		} /* end: switch */
	}
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
