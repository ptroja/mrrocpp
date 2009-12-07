/*
 * ecp_vis_ib_eih_planar_irp6ot.cc
 *
 *  Created on: Dec 9, 2008
 *      Author: pwilkows
 */

#include "ecp/irp6_on_track/ecp_g_rotate_gripper.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

ecp_g_rotate_gripper::ecp_g_rotate_gripper(common::task::task& _ecp_task, double _speed):
	common::generator::generator(_ecp_task) {
	speed = _speed;

	td.internode_step_no = 30;
	td.value_in_step_no = 29;
	td.arm_type = lib::JOINT;

	lastStep = false;
}

ecp_g_rotate_gripper::~ecp_g_rotate_gripper() {

}



bool ecp_g_rotate_gripper::first_step() {
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];
	vsp_fradia->to_vsp.haar_detect_mode = lib::PERFORM_ROTATION;
	ecp_t.sr_ecp_msg->message("first_step");

    the_robot->ecp_command.instruction.instruction_type = lib::GET;
    the_robot->EDP_data.get_type = ARM_DV;
    the_robot->EDP_data.get_arm_type = td.arm_type;
    the_robot->EDP_data.motion_type = lib::RELATIVE;
    the_robot->EDP_data.next_interpolation_type = lib::MIM;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

	return true;
}

bool ecp_g_rotate_gripper::next_step() {
	ecp_t.sr_ecp_msg->message("Next step");

	if( lastStep ){
		std::cout<<"Last step\n";
		return false;
	}

	double angle;
	lib::HD_READING state;


	//Sprwadz czy otrzymano rozwiazanie od VSP.
	lib::VSP_REPORT vsp_report = vsp_fradia->from_vsp.vsp_report;
	if (vsp_report == lib::VSP_REPLY_OK) {
		ecp_t.sr_ecp_msg->message("Weszlo do  VSP_REP_OK\n");
		state = vsp_fradia->from_vsp.comm_image.sensor_union.hd_angle.reading_state;
		if(state == lib::HD_SOLUTION_NOTFOUND){
			std::cout<<"Weszlo do HD_SOLUTION_NOTFOUND\n";
			ecp_t.sr_ecp_msg->message("Weszlo do HD_SOLUTION_NOTFOUND\n");
			vsp_fradia->to_vsp.haar_detect_mode = lib::WITHOUT_ROTATION;
			return false;
	    }
		else if(state == lib::HD_SOLUTION_FOUND){
			angle = vsp_fradia->from_vsp.comm_image.sensor_union.hd_angle.angle;
			//Obliczam ile krokow.
			ecp_t.sr_ecp_msg->message("Weszlo do HD_SOLUTION_FOUND\n");
			td.internode_step_no = (int)((angle/speed)/0.002);
			std::cout<<"angle: "<<angle<<std::endl;
			std::cout<<"motionsteps: "<<td.internode_step_no<<std::endl;
			vsp_fradia->to_vsp.haar_detect_mode = lib::WITHOUT_ROTATION;
			lastStep = true;
		}
	}else if (vsp_report == lib::VSP_READING_NOT_READY){
		angle = 0.0;
		td.internode_step_no = 30;
		//Rotacja sie wykonuje nastepny krok bez rotacji.
		vsp_fradia->to_vsp.haar_detect_mode = lib::WITHOUT_ROTATION;
		std::cout<<"Weszlo do VSP_READING_NOT_READY\n";
		ecp_t.sr_ecp_msg->message("Weszlo do VSP_READING_NOT_READY\n");
	}
	else{
		angle = 0.0;
		td.internode_step_no = 30;
		vsp_fradia->to_vsp.haar_detect_mode = lib::PERFORM_ROTATION;
		std::cout<<"Weszlo do NOT VSP_REP_OK\n";
		ecp_t.sr_ecp_msg->message("Weszlo do NOT VSP_REP_OK\n");
	}

    the_robot->ecp_command.instruction.instruction_type = lib::SET;
    the_robot->EDP_data.set_type = ARM_DV;
    the_robot->EDP_data.get_type = NOTHING_DV;
    the_robot->EDP_data.motion_type = lib::RELATIVE;
    the_robot->EDP_data.get_arm_type = td.arm_type;
  //  td.internode_step_no = (2000*angle)/0.55;
    the_robot->EDP_data.motion_steps = td.internode_step_no;
    the_robot->EDP_data.value_in_step_no = td.value_in_step_no-1;
    the_robot->EDP_data.next_joint_arm_coordinates[0] = 0.0;
    the_robot->EDP_data.next_joint_arm_coordinates[1] = 0.0;
    the_robot->EDP_data.next_joint_arm_coordinates[2] = 0.0;
    the_robot->EDP_data.next_joint_arm_coordinates[3] = 0.0;
    the_robot->EDP_data.next_joint_arm_coordinates[4] = 0.0;
    the_robot->EDP_data.next_joint_arm_coordinates[5] = 0.0;
    the_robot->EDP_data.next_joint_arm_coordinates[6] = angle;
    the_robot->EDP_data.next_joint_arm_coordinates[7] = 0.0;

	return true;
}

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


