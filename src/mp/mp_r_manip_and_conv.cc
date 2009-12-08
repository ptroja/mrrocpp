#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/mis_fun.h"
#include "lib/srlib.h"
#include "mp/mp_r_manip_and_conv.h"

namespace mrrocpp {
namespace mp {
namespace robot {

manip_and_conv::manip_and_conv(lib::robot_name_t l_robot_name, const char* _section_name, task::task &mp_object_l) :
	robot(l_robot_name, _section_name, mp_object_l), servos_number(0), has_gripper(false)
{
	switch (l_robot_name) {
		case lib::ROBOT_IRP6_ON_TRACK:
			servos_number = IRP6_ON_TRACK_NUM_OF_SERVOS;
			has_gripper = true;
			break;
		case lib::ROBOT_IRP6_POSTUMENT:
			servos_number = IRP6_POSTUMENT_NUM_OF_SERVOS;
			has_gripper = true;
			break;
		case lib::ROBOT_CONVEYOR:
			servos_number = CONVEYOR_NUM_OF_SERVOS;
			break;
		case lib::ROBOT_IRP6_MECHATRONIKA:
			servos_number = IRP6_MECHATRONIKA_NUM_OF_SERVOS;
			break;
		default: // error
			break;
	}
}

void manip_and_conv::create_next_pose_command(void)
{
	// wypelnia bufor wysylkowy do ECP na podstawie danych
	// zawartych w skladowych generatora lub warunku

	mp_command.instruction.instruction_type = mp_command.instruction.instruction_type;
	mp_command.instruction.set_type = mp_command.instruction.set_type;
	mp_command.instruction.get_type = mp_command.instruction.get_type;
	mp_command.instruction.set_rmodel_type = mp_command.instruction.set_rmodel_type;
	mp_command.instruction.get_rmodel_type = mp_command.instruction.get_rmodel_type;
	mp_command.instruction.set_arm_type = mp_command.instruction.set_arm_type;
	mp_command.instruction.get_arm_type = mp_command.instruction.get_arm_type;
	mp_command.instruction.output_values = mp_command.instruction.output_values;
	mp_command.instruction.interpolation_type = mp_command.instruction.interpolation_type;

	switch (mp_command.instruction.instruction_type) {
		case lib::SET:
		case lib::SET_GET:
			if (mp_command.instruction.set_type & RMODEL_DV) {
				switch (mp_command.instruction.set_rmodel_type) {
					case lib::TOOL_FRAME:
						lib::copy_frame(mp_command.instruction.rmodel.tool_frame_def.tool_frame, mp_command.instruction.rmodel.tool_frame_def.tool_frame);
						break;
					case lib::TOOL_XYZ_ANGLE_AXIS:
						for (int j=0; j<6; j++) {
							mp_command.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
									= mp_command.instruction.rmodel.tool_coordinate_def.tool_coordinates[j];
						}
						break;
					case lib::TOOL_XYZ_EULER_ZYZ:
						for (int j=0; j<6; j++) {
							mp_command.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
									= mp_command.instruction.rmodel.tool_coordinate_def.tool_coordinates[j];
						}
						break;
					case lib::TOOL_AS_XYZ_EULER_ZY:
						if (robot_name != lib::ROBOT_IRP6_POSTUMENT && robot_name != lib::ROBOT_IRP6_MECHATRONIKA) {
							throw MP_error(lib::NON_FATAL_ERROR, INVALID_RMODEL_TYPE);
						}
						for (int j=0; j<6; j++) {
							mp_command.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
									= mp_command.instruction.rmodel.tool_coordinate_def.tool_coordinates[j];
						}
						break;
					case lib::ARM_KINEMATIC_MODEL:
						mp_command.instruction.rmodel.kinematic_model.kinematic_model_no
								= mp_command.instruction.rmodel.kinematic_model.kinematic_model_no;
						break;
					case lib::SERVO_ALGORITHM:
						for (int j=0; j<servos_number; j++) {
							mp_command.instruction.rmodel.servo_algorithm.servo_algorithm_no[j]
									= mp_command.instruction.rmodel.servo_algorithm.servo_algorithm_no[j];
							mp_command.instruction.rmodel.servo_algorithm.servo_parameters_no[j]
									= mp_command.instruction.rmodel.servo_algorithm.servo_parameters_no[j];
						}
						break;
					case lib::FORCE_TOOL:
						for (int j=0; j<3; j++) {
							mp_command.instruction.rmodel.force_tool.position[j]
									= mp_command.instruction.rmodel.force_tool.position[j];
						}
						mp_command.instruction.rmodel.force_tool.weight = mp_command.instruction.rmodel.force_tool.weight;
						break;
					case lib::FORCE_BIAS:
						break;
					default: // Blad: niewlasciwy typ modelu robota
						throw MP_error(lib::NON_FATAL_ERROR, INVALID_RMODEL_TYPE);
				}
			}

			if (mp_command.instruction.set_type & ARM_DV) { // ramie
				mp_command.instruction.motion_type = mp_command.instruction.motion_type;

				mp_command.instruction.motion_steps = mp_command.instruction.motion_steps;
				mp_command.instruction.value_in_step_no = mp_command.instruction.value_in_step_no;
				// Wypelniamy czesc zwiazana z polozeniem ramienia
				switch (mp_command.instruction.set_arm_type) {
					case lib::FRAME:
						lib::copy_frame(mp_command.instruction.arm.pf_def.arm_frame, mp_command.instruction.arm.pf_def.arm_frame);

						break;
					case lib::XYZ_ANGLE_AXIS:
						for (int j=0; j<6; j++) {
							mp_command.instruction.arm.pf_def.arm_coordinates[j]
									= mp_command.instruction.arm.pf_def.arm_coordinates[j];
						}

						break;
					case lib::XYZ_EULER_ZYZ:
						for (int j=0; j<6; j++) {
							mp_command.instruction.arm.pf_def.arm_coordinates[j]
									= mp_command.instruction.arm.pf_def.arm_coordinates[j];
						}

						break;
					case lib::JOINT:
						for (int j=0; j<servos_number; j++) {
							mp_command.instruction.arm.pf_def.arm_coordinates[j]
									= mp_command.instruction.arm.pf_def.arm_coordinates[j];
						}
						break;
					case lib::MOTOR:
						for (int j=0; j<servos_number; j++) {
							mp_command.instruction.arm.pf_def.arm_coordinates[j]
									= mp_command.instruction.arm.pf_def.arm_coordinates[j];
						}
					case lib::PF_VELOCITY:
						for (int j=0; j<servos_number; j++) {
							mp_command.instruction.arm.pf_def.arm_coordinates[j] = mp_command.instruction.arm.pf_def.arm_coordinates[j];
						}
						break;
					default: // Blad: niewlasciwy sposob zadawania polozenia ramienia
						throw MP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
				}

				switch (mp_command.instruction.interpolation_type) {
					case lib::MIM:
						break;
					case lib::TCIM:
						for (int i=0; i<6; i++) {
							mp_command.instruction.arm.pf_def.inertia[i] =mp_command.instruction.arm.pf_def.inertia[i];
							mp_command.instruction.arm.pf_def.reciprocal_damping[i]
									=mp_command.instruction.arm.pf_def.reciprocal_damping[i];
							mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i]
									=mp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i];
							mp_command.instruction.arm.pf_def.behaviour[i] = mp_command.instruction.arm.pf_def.behaviour[i]; // pozycja docelowa
						}
						break;
					default:
						throw MP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
						break;
				}

				if (has_gripper) {
					mp_command.instruction.arm.pf_def.gripper_coordinate = mp_command.instruction.arm.pf_def.gripper_coordinate; // zadany stopien rozwarcia chwytaka
				}

			}
			break;
		case lib::GET:
		case lib::SYNCHRO:
		case lib::QUERY:
			break;
		default: // blad: nieprawidlowe polecenie
			throw MP_error (lib::NON_FATAL_ERROR, INVALID_ECP_COMMAND);
	}
}

void manip_and_conv::get_reply(void)
{
	// pobiera z pakietu przeslanego z ECP informacje i wstawia je do
	// odpowiednich skladowych generatora lub warunku
	ecp_reply_package.reply = ecp_reply_package.reply;
	ecp_reply_package.reply_package.reply_type = ecp_reply_package.reply_package.reply_type;
	switch (ecp_reply_package.reply_package.reply_type) {
		case lib::ERROR:
			ecp_reply_package.reply_package.error_no.error0 = ecp_reply_package.reply_package.error_no.error0;
			ecp_reply_package.reply_package.error_no.error1 = ecp_reply_package.reply_package.error_no.error1;
			break;
		case lib::ACKNOWLEDGE:
			break;
		case lib::SYNCHRO_OK:
			break;
		case lib::ARM_INPUTS:
			get_input_reply();
		case lib::ARM:
			get_arm_reply();
			break;
		case lib::RMODEL_INPUTS:
			get_input_reply();
		case lib::RMODEL:
			get_rmodel_reply();
			break;
		case lib::INPUTS:
			get_input_reply();
			break;
		case lib::ARM_RMODEL_INPUTS:
			get_input_reply();
		case lib::ARM_RMODEL:
			get_arm_reply();
			get_rmodel_reply();
			break;
		default: // bledna przesylka
			throw MP_error (lib::NON_FATAL_ERROR, INVALID_EDP_REPLY);
	}
}

void manip_and_conv::get_input_reply(void)
{
	ecp_reply_package.reply_package.input_values = ecp_reply_package.reply_package.input_values;
	for (int i=0; i<8; i++) {
		ecp_reply_package.reply_package.analog_input[i]=ecp_reply_package.reply_package.analog_input[i];
	}
}

void manip_and_conv::get_arm_reply(void)
{
	switch (ecp_reply_package.reply_package.arm_type) {
		case lib::MOTOR:
			for (int i=0; i<servos_number; i++) {
				ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i]
						= ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i];
			}

			break;
		case lib::JOINT:
			for (int i=0; i<servos_number; i++) {
				ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i]
						= ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i];
			}

			break;
		case lib::FRAME:
			if (robot_name == lib::ROBOT_CONVEYOR)
				throw MP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			lib::copy_frame(ecp_reply_package.reply_package.arm.pf_def.arm_frame, ecp_reply_package.reply_package.arm.pf_def.arm_frame);

			break;
		case lib::XYZ_EULER_ZYZ:
			if (robot_name == lib::ROBOT_CONVEYOR)
				throw MP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			for (int i=0; i<6; i++) {
				ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i]
						= ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i];
			}

			break;
		case lib::XYZ_ANGLE_AXIS:
			if (robot_name == lib::ROBOT_CONVEYOR)
				throw MP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			for (int i=0; i<6; i++) {
				ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i]
						= ecp_reply_package.reply_package.arm.pf_def.arm_coordinates[i];
			}

			break;
		default: // bledny typ specyfikacji pozycji
			throw MP_error (lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}

	for (int i = 0; i<6; i++) {
		ecp_reply_package.reply_package.arm.pf_def.force_xyz_torque_xyz[i]
				= ecp_reply_package.reply_package.arm.pf_def.force_xyz_torque_xyz[i];
	}

	if (has_gripper) {
		ecp_reply_package.reply_package.arm.pf_def.gripper_reg_state = ecp_reply_package.reply_package.arm.pf_def.gripper_reg_state;
		ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate = ecp_reply_package.reply_package.arm.pf_def.gripper_coordinate;
	}

}

void manip_and_conv::get_rmodel_reply(void)
{
	switch (ecp_reply_package.reply_package.rmodel_type) {
		case lib::TOOL_FRAME:
			if (robot_name != lib::ROBOT_IRP6_POSTUMENT && robot_name != lib::ROBOT_IRP6_ON_TRACK && robot_name
					!= lib::ROBOT_IRP6_MECHATRONIKA) {
				throw MP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
			lib::copy_frame(ecp_reply_package.reply_package.rmodel.tool_frame_def.tool_frame, ecp_reply_package.reply_package.rmodel.tool_frame_def.tool_frame);
			break;
		case lib::TOOL_XYZ_ANGLE_AXIS:
			if (robot_name != lib::ROBOT_IRP6_POSTUMENT && robot_name != lib::ROBOT_IRP6_ON_TRACK && robot_name
					!= lib::ROBOT_IRP6_MECHATRONIKA) {
				throw MP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
			for (int i=0; i<6; i++) {
				ecp_reply_package.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i]
						= ecp_reply_package.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			}
			break;
		case lib::TOOL_XYZ_EULER_ZYZ:
			if (robot_name != lib::ROBOT_IRP6_POSTUMENT && robot_name != lib::ROBOT_IRP6_ON_TRACK && robot_name
					!= lib::ROBOT_IRP6_MECHATRONIKA) {
				throw MP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
			for (int i=0; i<6; i++) {
				ecp_reply_package.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i]
						= ecp_reply_package.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			}
			break;
		case lib::TOOL_AS_XYZ_EULER_ZY:
			if (robot_name != lib::ROBOT_IRP6_POSTUMENT && robot_name != lib::ROBOT_IRP6_MECHATRONIKA) {
				throw MP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
			}
			for (int i=0; i<6; i++) {
				ecp_reply_package.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i]
						= ecp_reply_package.reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			}
			break;
		case lib::ARM_KINEMATIC_MODEL:
			ecp_reply_package.reply_package.rmodel.kinematic_model.kinematic_model_no
					= ecp_reply_package.reply_package.rmodel.kinematic_model.kinematic_model_no;
			break;
		case lib::SERVO_ALGORITHM:
			for (int j=0; j<servos_number; j++) {
				ecp_reply_package.reply_package.rmodel.servo_algorithm.servo_algorithm_no[j]
						= ecp_reply_package.reply_package.rmodel.servo_algorithm.servo_algorithm_no[j];
				ecp_reply_package.reply_package.rmodel.servo_algorithm.servo_parameters_no[j]
						= ecp_reply_package.reply_package.rmodel.servo_algorithm.servo_parameters_no[j];
			}
			break;
		case lib::FORCE_TOOL:
			for (int j=0; j<3; j++) {
				ecp_reply_package.reply_package.rmodel.force_tool.position[j]
						= ecp_reply_package.reply_package.rmodel.force_tool.position[j];
			}
			ecp_reply_package.reply_package.rmodel.force_tool.weight = ecp_reply_package.reply_package.rmodel.force_tool.weight;
			break;
		default: // bledny typ specyfikacji modelu robota
			throw MP_error(lib::NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	}
}
} // namespace robot
} // namespace mp
} // namespace mrrocpp
