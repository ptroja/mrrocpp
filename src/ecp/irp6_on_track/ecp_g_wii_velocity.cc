#include "ecp/irp6_on_track/ecp_g_wii_velocity.h"

#include "common/impconst.h"
#include "common/com_buf.h"
#include "math.h"

ecp_wii_velocity_generator::ecp_wii_velocity_generator (ecp_task& _ecp_task) : ecp_tff_nose_run_generator(_ecp_task)
{
	step_no = 0;
	// domyslnie wszytkie osie podatne a pulse_check nieaktywne
	configure_behaviour(UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION, UNGUARDED_MOTION);
	configure_pulse_check (false);
	configure_velocity (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	configure_force (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	configure_reciprocal_damping (FORCE_RECIPROCAL_DAMPING, FORCE_RECIPROCAL_DAMPING, FORCE_RECIPROCAL_DAMPING,
		 TORQUE_RECIPROCAL_DAMPING, TORQUE_RECIPROCAL_DAMPING, TORQUE_RECIPROCAL_DAMPING);
	configure_inertia (FORCE_INERTIA, FORCE_INERTIA, FORCE_INERTIA, TORQUE_INERTIA, TORQUE_INERTIA, TORQUE_INERTIA);

	set_force_meassure (false);
}

bool ecp_wii_velocity_generator::first_step()
{
	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);

	the_robot->EDP_data.instruction_type = GET;
	the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
	the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
	the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
	the_robot->EDP_data.set_arm_type = PF_VELOCITY;
	the_robot->EDP_data.get_arm_type = FRAME;
	the_robot->EDP_data.motion_type = RELATIVE;
	the_robot->EDP_data.next_interpolation_type = TCIM;
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

bool ecp_wii_velocity_generator::next_step()
{
	char buffer[200];
	try
	{
		sensor_m[SENSOR_WIIMOTE]->get_reading();
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
	the_robot->EDP_data.instruction_type = SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	if (node_counter==1)
	{
		the_robot->EDP_data.next_gripper_coordinate
				= the_robot->EDP_data.current_gripper_coordinate;
	}

	return true;
}
