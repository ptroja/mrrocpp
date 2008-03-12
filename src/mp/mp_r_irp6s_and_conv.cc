#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp_r_irp6s_and_conv.h"

mp_irp6s_and_conv_robot::mp_irp6s_and_conv_robot(ROBOT_ENUM l_robot_name, const char* _section_name, mp_task &mp_object_l) :
	mp_robot(l_robot_name, _section_name, mp_object_l), servos_number(0), has_gripper(false)
{
	switch (l_robot_name) {
		case ROBOT_IRP6_ON_TRACK:
			servos_number = IRP6_ON_TRACK_NUM_OF_SERVOS;
			has_gripper = true;
			break;
		case ROBOT_IRP6_POSTUMENT:
			servos_number = IRP6_POSTUMENT_NUM_OF_SERVOS;
			has_gripper = true;
			break;
		case ROBOT_CONVEYOR:
			servos_number = CONVEYOR_NUM_OF_SERVOS;
			break;
		case ROBOT_IRP6_MECHATRONIKA:
			servos_number = IRP6_MECHATRONIKA_NUM_OF_SERVOS;
			break;
		default: // error
			break;
	}
}

void mp_irp6s_and_conv_robot::create_next_pose_command(void)
{
	// wypelnia bufor wysylkowy do ECP na podstawie danych
	// zawartych w skladowych generatora lub warunku

	mp_command.mp_package.instruction.instruction_type = ecp_td.instruction_type;
	mp_command.mp_package.instruction.set_type = ecp_td.set_type;
	mp_command.mp_package.instruction.get_type = ecp_td.get_type;
	mp_command.mp_package.instruction.set_rmodel_type = ecp_td.set_rmodel_type;
	mp_command.mp_package.instruction.get_rmodel_type = ecp_td.get_rmodel_type;
	mp_command.mp_package.instruction.set_arm_type = ecp_td.set_arm_type;
	mp_command.mp_package.instruction.get_arm_type = ecp_td.get_arm_type;
	mp_command.mp_package.instruction.output_values = ecp_td.output_values;
	mp_command.mp_package.instruction.interpolation_type = ecp_td.next_interpolation_type;
	switch (ecp_td.instruction_type) {
		case SET:
		case SET_GET:
			if (ecp_td.set_type & RMODEL_DV) {
				switch (ecp_td.set_rmodel_type) {
					case TOOL_FRAME:
						copy_frame(mp_command.mp_package.instruction.rmodel.tool_frame_def.tool_frame, ecp_td.next_tool_frame);
						break;
					case TOOL_XYZ_ANGLE_AXIS:
						for (int j=0; j<6; j++) {
							mp_command.mp_package.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
									= ecp_td.next_XYZ_AA_tool_coordinates[j];
						}
						break;
					case TOOL_XYZ_EULER_ZYZ:
						for (int j=0; j<6; j++) {
							mp_command.mp_package.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
									= ecp_td.next_XYZ_ZYZ_tool_coordinates[j];
						}
						break;
					case TOOL_AS_XYZ_EULER_ZY:
						if (robot_name != ROBOT_IRP6_POSTUMENT && robot_name != ROBOT_IRP6_MECHATRONIKA) {
							throw MP_error(NON_FATAL_ERROR, INVALID_RMODEL_TYPE);
						}
						for (int j=0; j<6; j++) {
							mp_command.mp_package.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
									= ecp_td.next_XYZ_ZYZ_tool_coordinates[j];
						}
						break;
					case ARM_KINEMATIC_MODEL:
						mp_command.mp_package.instruction.rmodel.kinematic_model.kinematic_model_no
								= ecp_td.next_kinematic_model_no;
						break;
					case SERVO_ALGORITHM:
						for (int j=0; j<servos_number; j++) {
							mp_command.mp_package.instruction.rmodel.servo_algorithm.servo_algorithm_no[j]
									= ecp_td.next_servo_algorithm_no[j];
							mp_command.mp_package.instruction.rmodel.servo_algorithm.servo_parameters_no[j]
									= ecp_td.next_servo_parameters_no[j];
						}
						break;
					case FORCE_TOOL:
						for (int j=0; j<3; j++) {
							mp_command.mp_package.instruction.rmodel.force_tool.position[j]
									= ecp_td.next_force_tool_position[j];
						}
						mp_command.mp_package.instruction.rmodel.force_tool.weight = ecp_td.next_force_tool_weight;
						break;
					case FORCE_BIAS:
						break;
					default: // Blad: niewlasciwy typ modelu robota
						throw MP_error(NON_FATAL_ERROR, INVALID_RMODEL_TYPE);
				}
			}

			if (ecp_td.set_type & ARM_DV) { // ramie
				mp_command.mp_package.instruction.motion_type = ecp_td.motion_type;

				mp_command.mp_package.instruction.motion_steps = ecp_td.motion_steps;
				mp_command.mp_package.instruction.value_in_step_no = ecp_td.value_in_step_no;
				// Wypelniamy czesc zwiazana z polozeniem ramienia
				switch (ecp_td.set_arm_type) {
					case FRAME:
						copy_frame(mp_command.mp_package.instruction.arm.pf_def.arm_frame, ecp_td.next_arm_frame);

						break;
					case XYZ_ANGLE_AXIS:
						for (int j=0; j<6; j++) {
							mp_command.mp_package.instruction.arm.pf_def.arm_coordinates[j]
									= ecp_td.next_XYZ_AA_arm_coordinates[j];
						}

						break;
					case XYZ_EULER_ZYZ:
						for (int j=0; j<6; j++) {
							mp_command.mp_package.instruction.arm.pf_def.arm_coordinates[j]
									= ecp_td.next_XYZ_ZYZ_arm_coordinates[j];
						}

						break;
					case JOINT:
						for (int j=0; j<servos_number; j++) {
							mp_command.mp_package.instruction.arm.pf_def.arm_coordinates[j]
									= ecp_td.next_joint_arm_coordinates[j];
						}
						break;
					case MOTOR:
						for (int j=0; j<servos_number; j++) {
							mp_command.mp_package.instruction.arm.pf_def.arm_coordinates[j]
									= ecp_td.next_motor_arm_coordinates[j];
						}
					case PF_VELOCITY:
						for (int j=0; j<servos_number; j++) {
							mp_command.mp_package.instruction.arm.pf_def.arm_coordinates[j] = ecp_td.next_velocity[j];
						}
						break;
					default: // Blad: niewlasciwy sposob zadawania polozenia ramienia
						throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
				}

				switch (ecp_td.next_interpolation_type) {
					case MOTORS_INTERPOLATION:
						break;
					case EXTERNAL_INTERPOLATION_WITH_FORCE:
						for (int i=0; i<6; i++) {
							mp_command.mp_package.instruction.arm.pf_def.inertia[i] =ecp_td.next_inertia[i];
							mp_command.mp_package.instruction.arm.pf_def.reciprocal_damping[i]
									=ecp_td.next_reciprocal_damping[i];
							mp_command.mp_package.instruction.arm.pf_def.force_xyz_torque_xyz[i]
									=ecp_td.next_force_xyz_torque_xyz[i];
							mp_command.mp_package.instruction.arm.pf_def.behaviour[i] = ecp_td.next_behaviour[i]; // pozycja docelowa
						}
						break;
					default:
						throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
						break;
				}

				if (has_gripper) {
					mp_command.mp_package.instruction.arm.pf_def.gripper_coordinate = ecp_td.next_gripper_coordinate; // zadany stopien rozwarcia chwytaka
				}

			}
			break;
		case GET:
		case SYNCHRO:
		case QUERY:
			break;
		default: // blad: nieprawidlowe polecenie
			throw MP_error (NON_FATAL_ERROR, INVALID_ECP_COMMAND);
	}
}

void mp_irp6s_and_conv_robot::get_reply(void)
{
	// pobiera z pakietu przeslanego z ECP informacje i wstawia je do
	// odpowiednich skladowych generatora lub warunku
	ecp_td.ecp_reply = ecp_reply_package.reply;
	ecp_td.reply_type = ecp_reply_package.ecp_reply.reply_package.reply_type;
	switch (ecp_td.reply_type) {
		case ERROR:
			ecp_td.error_no.error0 = ecp_reply_package.ecp_reply.reply_package.error_no.error0;
			ecp_td.error_no.error1 = ecp_reply_package.ecp_reply.reply_package.error_no.error1;
			break;
		case ACKNOWLEDGE:
			break;
		case SYNCHRO_OK:
			break;
		case ARM_INPUTS:
			get_input_reply();
		case ARM:
			get_arm_reply();
			break;
		case RMODEL_INPUTS:
			get_input_reply();
		case RMODEL:
			get_rmodel_reply();
			break;
		case INPUTS:
			get_input_reply();
			break;
		case ARM_RMODEL_INPUTS:
			get_input_reply();
		case ARM_RMODEL:
			get_arm_reply();
			get_rmodel_reply();
			break;
		default: // bledna przesylka
			throw MP_error (NON_FATAL_ERROR, INVALID_EDP_REPLY);
	}
}

void mp_irp6s_and_conv_robot::get_input_reply(void)
{
	ecp_td.input_values = ecp_reply_package.ecp_reply.reply_package.input_values;
	for (int i=0; i<8; i++) {
		ecp_td.analog_input[i]=ecp_reply_package.ecp_reply.reply_package.analog_input[i];
	}
}

void mp_irp6s_and_conv_robot::get_arm_reply(void)
{
	switch (ecp_reply_package.ecp_reply.reply_package.arm_type) {
		case MOTOR:
			for (int i=0; i<servos_number; i++) {
				ecp_td.current_motor_arm_coordinates[i]
						= ecp_reply_package.ecp_reply.reply_package.arm.pf_def.arm_coordinates[i];
			}

			break;
		case JOINT:
			for (int i=0; i<servos_number; i++) {
				ecp_td.current_joint_arm_coordinates[i]
						= ecp_reply_package.ecp_reply.reply_package.arm.pf_def.arm_coordinates[i];
			}

			break;
		case FRAME:
			if (robot_name == ROBOT_CONVEYOR)
				throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			copy_frame(ecp_td.current_arm_frame, ecp_reply_package.ecp_reply.reply_package.arm.pf_def.arm_frame);

			break;
		case XYZ_EULER_ZYZ:
			if (robot_name == ROBOT_CONVEYOR)
				throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			for (int i=0; i<6; i++) {
				ecp_td.current_XYZ_ZYZ_arm_coordinates[i]
						= ecp_reply_package.ecp_reply.reply_package.arm.pf_def.arm_coordinates[i];
			}

			break;
		case XYZ_ANGLE_AXIS:
			if (robot_name == ROBOT_CONVEYOR)
				throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			for (int i=0; i<6; i++) {
				ecp_td.current_XYZ_AA_arm_coordinates[i]
						= ecp_reply_package.ecp_reply.reply_package.arm.pf_def.arm_coordinates[i];
			}

			break;
		default: // bledny typ specyfikacji pozycji
			throw MP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}

	for (int i = 0; i<6; i++) {
		ecp_td.current_force_xyz_torque_xyz[i]
				= ecp_reply_package.ecp_reply.reply_package.arm.pf_def.force_xyz_torque_xyz[i];
	}

	if (has_gripper) {
		ecp_td.gripper_reg_state = ecp_reply_package.ecp_reply.reply_package.arm.pf_def.gripper_reg_state;
		ecp_td.current_gripper_coordinate = ecp_reply_package.ecp_reply.reply_package.arm.pf_def.gripper_coordinate;
	}

}

void mp_irp6s_and_conv_robot::get_rmodel_reply(void)
{
	switch (ecp_reply_package.ecp_reply.reply_package.rmodel_type) {
		case TOOL_FRAME:
			if (robot_name != ROBOT_IRP6_POSTUMENT && robot_name != ROBOT_IRP6_ON_TRACK && robot_name
					!= ROBOT_IRP6_MECHATRONIKA) {
				throw MP_error(NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
			copy_frame(ecp_td.current_tool_frame, ecp_reply_package.ecp_reply.reply_package.rmodel.tool_frame_def.tool_frame);
			break;
		case TOOL_XYZ_ANGLE_AXIS:
			if (robot_name != ROBOT_IRP6_POSTUMENT && robot_name != ROBOT_IRP6_ON_TRACK && robot_name
					!= ROBOT_IRP6_MECHATRONIKA) {
				throw MP_error(NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
			for (int i=0; i<6; i++) {
				ecp_td.current_XYZ_AA_tool_coordinates[i]
						= ecp_reply_package.ecp_reply.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			}
			break;
		case TOOL_XYZ_EULER_ZYZ:
			if (robot_name != ROBOT_IRP6_POSTUMENT && robot_name != ROBOT_IRP6_ON_TRACK && robot_name
					!= ROBOT_IRP6_MECHATRONIKA) {
				throw MP_error(NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
			for (int i=0; i<6; i++) {
				ecp_td.current_XYZ_ZYZ_tool_coordinates[i]
						= ecp_reply_package.ecp_reply.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			}
			break;
		case TOOL_AS_XYZ_EULER_ZY:
			if (robot_name != ROBOT_IRP6_POSTUMENT && robot_name != ROBOT_IRP6_MECHATRONIKA) {
				throw MP_error(NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
			for (int i=0; i<6; i++) {
				ecp_td.current_XYZ_ZYZ_tool_coordinates[i]
						= ecp_reply_package.ecp_reply.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			}
			break;
		case ARM_KINEMATIC_MODEL:
			ecp_td.current_kinematic_model_no
					= ecp_reply_package.ecp_reply.reply_package.rmodel.kinematic_model.kinematic_model_no;
			break;
		case SERVO_ALGORITHM:
			for (int j=0; j<servos_number; j++) {
				ecp_td.current_servo_algorithm_no[j]
						= ecp_reply_package.ecp_reply.reply_package.rmodel.servo_algorithm.servo_algorithm_no[j];
				ecp_td.current_servo_parameters_no[j]
						= ecp_reply_package.ecp_reply.reply_package.rmodel.servo_algorithm.servo_parameters_no[j];
			}
			break;
		case FORCE_TOOL:
			for (int j=0; j<3; j++) {
				ecp_td.current_force_tool_position[j]
						= ecp_reply_package.ecp_reply.reply_package.rmodel.force_tool.position[j];
			}
			ecp_td.current_force_tool_weight = ecp_reply_package.ecp_reply.reply_package.rmodel.force_tool.weight;
			break;
		default: // bledny typ specyfikacji modelu robota
			throw MP_error(NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}
}
