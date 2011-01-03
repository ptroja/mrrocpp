/*
 * generator/ecp_g_vis_ib_eih_planar_irp6ot.cc
 *
 *  Created on: Dec 9, 2008
 *      Author: pwilkows
 */

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_rotate_gripper.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {

ecp_g_rotate_gripper::ecp_g_rotate_gripper(common::task::task& _ecp_task, double _speed) :
	common::generator::generator(_ecp_task)
{
	speed = _speed;

	td.internode_step_no = 30;
	td.value_in_step_no = 29;
	td.arm_type = lib::ECP_JOINT;

	lastStep = false;
}

ecp_g_rotate_gripper::~ecp_g_rotate_gripper()
{

}

bool ecp_g_rotate_gripper::first_step()
{
	vsp_fradia = dynamic_cast <fradia_sensor_haar_detect *> (sensor_m[ecp_mp::sensor::SENSOR_FRADIA]);

	vsp_fradia->set_initiate_message(PERFORM_ROTATION);

	sr_ecp_msg.message("first_step");

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	return true;
}

bool ecp_g_rotate_gripper::next_step()
{
	sr_ecp_msg.message("Next step");

	if (lastStep) {
		std::cout << "Last step\n";
		return false;
	}

	double angle = 0.0;
	HD_READING state;

	//Sprwadz czy otrzymano rozwiazanie od VSP.
	lib::sensor::VSP_REPORT_t vsp_report = vsp_fradia->get_report();
	if (vsp_report == lib::sensor::VSP_REPLY_OK) {
		sr_ecp_msg.message("Weszlo do  VSP_REP_OK\n");
		state = vsp_fradia->get_reading_message().reading_state;
		if (state == HD_SOLUTION_NOTFOUND) {
			std::cout << "Weszlo do HD_SOLUTION_NOTFOUND\n";
			sr_ecp_msg.message("Weszlo do HD_SOLUTION_NOTFOUND\n");
			vsp_fradia->set_initiate_message(WITHOUT_ROTATION);
			return false;
		} else if (state == HD_SOLUTION_FOUND) {
			angle = vsp_fradia->get_reading_message().angle;
			//Obliczam ile krokow.
			sr_ecp_msg.message("Weszlo do HD_SOLUTION_FOUND\n");
			td.internode_step_no = (int) ((angle / speed) / 0.002);
			std::cout << "angle: " << angle << std::endl;
			std::cout << "motionsteps: " << td.internode_step_no << std::endl;
			vsp_fradia->set_initiate_message(WITHOUT_ROTATION);
			lastStep = true;
		}
	} else if (vsp_report == lib::sensor::VSP_READING_NOT_READY) {
		angle = 0.0;
		td.internode_step_no = 30;
		//Rotacja sie wykonuje nastepny krok bez rotacji.
		vsp_fradia->set_initiate_message(WITHOUT_ROTATION);
		std::cout << "Weszlo do VSP_READING_NOT_READY\n";
		sr_ecp_msg.message("Weszlo do VSP_READING_NOT_READY\n");
	} else {
		angle = 0.0;
		td.internode_step_no = 30;
		vsp_fradia->set_initiate_message(WITHOUT_ROTATION);
		std::cout << "Weszlo do NOT VSP_REP_OK\n";
		sr_ecp_msg.message("Weszlo do NOT VSP_REP_OK\n");
	}

	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_type = NOTHING_DEFINITION;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
	//  td.internode_step_no = (2000*angle)/0.55;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no - 1;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[3] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[4] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[5] = 0.0;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[6] = angle;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[7] = 0.0;

	return true;
}

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
