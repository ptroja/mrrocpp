/*!
 * @file
 * @brief File contains mp delta and tight coop generators definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <cstring>

#include <boost/foreach.hpp>

#include "base/mp/mp_robot.h"
#include "base/mp/mp_task.h"

#include "generator/mp/mp_g_delta_tight_coop.h"

namespace mrrocpp {
namespace mp {
namespace generator {

delta::delta(task::task& _mp_task) :
		generator(_mp_task)
{
}

// ####################################################################################################
// Generator prostoliniowy o zadany przyrost polozenia/orientacji
// ####################################################################################################

tight_coop::tight_coop(task::task& _mp_task, lib::trajectory_description irp6ot_tr_des, lib::trajectory_description irp6p_tr_des) :
		delta(_mp_task)
{
	irp6ot_td = irp6ot_tr_des;
	irp6p_td = irp6p_tr_des;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tight_coop::first_step()
{

	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana

	idle_step_counter = 2;

	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				robot_node.second->mp_command.command = lib::NEXT_POSE;
				robot_node.second->mp_command.instruction.instruction_type = lib::GET;
				robot_node.second->mp_command.instruction.get_type = ARM_DEFINITION;
				robot_node.second->mp_command.instruction.set_type = ARM_DEFINITION;
				//robot_node.second->mp_command.instruction.set_arm_type = lib::XYZ_EULER_ZYZ;
				//robot_node.second->mp_command.instruction.get_arm_type = lib::XYZ_EULER_ZYZ;
				robot_node.second->mp_command.instruction.motion_type = lib::ABSOLUTE;
				robot_node.second->mp_command.instruction.interpolation_type = lib::MIM;
				robot_node.second->mp_command.instruction.motion_steps = irp6ot_td.internode_step_no;
				robot_node.second->mp_command.instruction.value_in_step_no = irp6ot_td.value_in_step_no;
				robot_node.second->communicate_with_ecp = true;
			}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step --------------------------------------
// ----------------------------------------------------------------------------------------------

bool tight_coop::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow

	if (idle_step_counter) { // Oczekiwanie na odczyt aktualnego polozenia koncowki
		idle_step_counter--;
		return true;
	}

	if (node_counter - 1 == irp6ot_td.interpolation_node_no)
		return false;

	common::robots_t::iterator robot_m_iterator = robot_m.begin();

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	robot_m_iterator->second->mp_command.instruction.instruction_type = lib::SET;
	robot_m_iterator->second->mp_command.instruction.get_type = NOTHING_DEFINITION;
	// TEMPORARY REMOVAL
	// robot_m_iterator->second->mp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
	//END OF TEMPORARY REMOVAL

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	// (okreslenie kolejnego wezla interpolacji)
	// i: licznik kolejnych wspolrzednych wektora [0..6]
	/* TEMPORARY REMOVAL
	 for (int i = 0; i < 6; i++) // zakladamy, ze na liscie jest jeden robot
	 robot_m_iterator->second->mp_command.instruction.arm.pf_def.arm_coordinates[i] =
	 robot_m_iterator->second->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i]
	 + node_counter * irp6ot_td.coordinate_delta[i] / irp6ot_td.interpolation_node_no;
	 */// END OF TEMPORARY REMOVAL
	 // printf("X_d= %lf  X_a= %lf\n",robot_list->E_ptr->mp_command.instruction.arm.pf_def.arm_coordinates[0],robot_list->E_ptr->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[0]);
	 // printf("Y_d= %lf  Y_a= %lf\n",robot_list->E_ptr->mp_command.instruction.arm.pf_def.arm_coordinates[1],robot_list->E_ptr->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[1]);
	 // printf("Z_d= %lf  Z_a= %lf\n",robot_list->E_ptr->mp_command.instruction.arm.pf_def.arm_coordinates[2],robot_list->E_ptr->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[2]);
	 // by Y - ZAKOMENTOWANE ponizej - nie wiadomo jaka idea temu przyswiecala
	 // ale dzialalo to zle z generatorami transparentnymi ECP
	/*
	 if (node_counter == td.interpolation_node_no) {
	 // Zakonczenie generacji trajektorii
	 robot_list->E_ptr->mp_command.command = lib::END_MOTION;
	 }
	 */

	if ((++robot_m_iterator) != robot_m.end()) {
		// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
		robot_m_iterator->second->mp_command.instruction.instruction_type = lib::SET;
		robot_m_iterator->second->mp_command.instruction.get_type = NOTHING_DEFINITION;
		/* TEMPORARY REMOVAL
		 robot_m_iterator->second->mp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
		 */ //END OFF TEMPORARY REMOVAL
		  // Obliczenie zadanej pozycji posredniej w tym kroku ruchu
		  // (okreslenie kolejnego wezla interpolacji)
		  // i: licznik kolejnych wspolrzednych wektora [0..6]
		/* TEMPORARY REMOVAL
		 for	(int i = 0; i < 6; i++) // zakladamy, ze na liscie jest jeden robot

		 robot_m_iterator->second->mp_command.instruction.arm.pf_def.arm_coordinates[i] =
		 robot_m_iterator->second->ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i]
		 + node_counter * irp6p_td.coordinate_delta[i] / irp6p_td.interpolation_node_no;
		 */ //END OFF TEMPORARY REMOVAL
	}

// skopiowac przygotowany rozkaz dla ECP do bufora wysylkowego

	return true;
}

} // namespace generator
} // namespace mp
} // namespace mrrocpp

