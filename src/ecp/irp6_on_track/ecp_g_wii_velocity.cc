#include "ecp/irp6_on_track/ecp_g_wii_velocity.h"

#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "math.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

sem_t sem;

wii_velocity::wii_velocity (common::task::base& _ecp_task) : common::generator::tff_nose_run(_ecp_task,10)
{
	configure_behaviour(lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION, lib::UNGUARDED_MOTION);
}

bool wii_velocity::first_step()
{
	sem_init(&sem,0,0);

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 3;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);

	the_robot->EDP_data.instruction_type = lib::GET;
	the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
	the_robot->EDP_data.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->EDP_data.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->EDP_data.set_arm_type = lib::PF_VELOCITY;
	the_robot->EDP_data.get_arm_type = lib::JOINT;
	the_robot->EDP_data.motion_type = lib::RELATIVE;
	the_robot->EDP_data.next_interpolation_type = lib::TCIM;
	the_robot->EDP_data.motion_steps = td.internode_step_no;
	the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		 the_robot->EDP_data.next_behaviour[i] = generator_edp_data.next_behaviour[i];
		 the_robot->EDP_data.next_velocity[i] = generator_edp_data.next_velocity[i];
		 the_robot->EDP_data.next_force_xyz_torque_xyz[i] = generator_edp_data.next_force_xyz_torque_xyz[i];
		 the_robot->EDP_data.next_reciprocal_damping[i] = generator_edp_data.next_reciprocal_damping[i];
		 the_robot->EDP_data.next_inertia[i] = generator_edp_data.next_inertia[i];
	}
	return true;
}

bool wii_velocity::next_step()
{
	char buffer[200];
	int operate = 0;
	
	try
	{
		sensor_m[lib::SENSOR_WIIMOTE]->get_reading();
	}
	catch(...)
	{
	}

	++step_no;

	if (pulse_check_activated && check_and_null_trigger())
	{
		return false;
	}

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->EDP_data.instruction_type = lib::SET_GET;

	operate = (int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.up || (int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.right || (int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.down || (int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.left;

	if(operate)
	{
		//wyznaczenie nowych wartosci predkosci
		configure_velocity(
			(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.up && !(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.right ? sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y * C_0 : 0,
			(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.right && !(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.up && !(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.down ? sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y * C_1 :	 0,
			(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.down && !(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.right ? sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y * C_2 : 0,
			(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.left ? sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y * C_3 : 0,
			(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.up && (int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.right ? sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y * C_4 : 0,
			(int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.down && (int)sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.right ? sensor_m[lib::SENSOR_WIIMOTE]->image.sensor_union.wiimote.orientation_y * C_5 : 0
		);
	}
	else
	{
		configure_velocity(0,0,0,0,0,0);
	}

	for (int i=0; i<6; i++)
	{
		 the_robot->EDP_data.next_velocity[i] = generator_edp_data.next_velocity[i];
	}
	
	//sprintf(buffer,"P%d %f %f %f %f %f %f",step_no,the_robot->EDP_data.current_joint_arm_coordinates[0],the_robot->EDP_data.current_joint_arm_coordinates[1],the_robot->EDP_data.current_joint_arm_coordinates[2],the_robot->EDP_data.current_joint_arm_coordinates[3],the_robot->EDP_data.current_joint_arm_coordinates[4],the_robot->EDP_data.current_joint_arm_coordinates[5]);
	//sprintf(buffer,"V%d %f %f %f %f	 %f %f",step_no,the_robot->EDP_data.next_velocity[0],the_robot->EDP_data.next_velocity[1],the_robot->EDP_data.next_velocity[2],the_robot->EDP_data.next_velocity[3],the_robot->EDP_data.next_velocity[4],the_robot->EDP_data.next_velocity[5]);
	//sr_ecp_msg.message(buffer);

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	if (node_counter==1)
	{
		the_robot->EDP_data.next_gripper_coordinate
				= the_robot->EDP_data.current_gripper_coordinate;
	}

	return true;
}
; // end: bool ecp_wii_velocity_generator::next_step ()

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

