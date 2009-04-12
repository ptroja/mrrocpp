// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (ECP) - methods
// Funkcje do tworzenia procesow ECP
// robot - irp6_postument
//
// -------------------------------------------------------------------------

#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mis_fun.h"

#include "ecp/irp6_postument/ecp_local.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {

ecp_irp6_postument_robot::ecp_irp6_postument_robot(configurator &_config, sr_ecp *_sr_ecp) :
	ecp_robot(ROBOT_IRP6_POSTUMENT, _config, _sr_ecp)
{
}
;
ecp_irp6_postument_robot::ecp_irp6_postument_robot(common::ecp_task& _ecp_object) :
	ecp_robot(ROBOT_IRP6_POSTUMENT, _ecp_object)
{
}
;

// --------------------------------------------------------------------------
void ecp_irp6_postument_robot::create_command(void)
{
	// wypelnia bufor wysylkowy do EDP na podstawie danych
	// zawartych w skladowych generatora lub warunku
	int j; // pomocnicze liczniki petli

	ecp_command.instruction.instruction_type = EDP_data.instruction_type;
	ecp_command.instruction.set_type = EDP_data.set_type;
	ecp_command.instruction.get_type = EDP_data.get_type;
	// printf("EDP_data.get_type: %d, ecp_command.instruction.get_type: %d\n",
	// EDP_data.get_type,ecp_command.instruction.get_type);

	ecp_command.instruction.set_rmodel_type = EDP_data.set_rmodel_type;
	ecp_command.instruction.get_rmodel_type = EDP_data.get_rmodel_type;
	ecp_command.instruction.set_arm_type = EDP_data.set_arm_type;
	ecp_command.instruction.get_arm_type = EDP_data.get_arm_type;
	ecp_command.instruction.output_values = EDP_data.output_values;

	ecp_command.instruction.interpolation_type = EDP_data.next_interpolation_type;

	switch (EDP_data.instruction_type) {
		case SET:
		case SET_GET:

			if (EDP_data.set_type & RMODEL_DV) {
				switch (EDP_data.set_rmodel_type) {
					case TOOL_FRAME:
						copy_frame(ecp_command.instruction.rmodel.tool_frame_def.tool_frame, EDP_data.next_tool_frame);
						break;
					case TOOL_XYZ_ANGLE_AXIS:
						for (j=0; j<6; j++)
							ecp_command.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
									= EDP_data.next_XYZ_AA_tool_coordinates[j];
						break;
					case TOOL_XYZ_EULER_ZYZ:
						for (j=0; j<6; j++)
							ecp_command.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
									= EDP_data.next_XYZ_ZYZ_tool_coordinates[j];
						break;
						///////////////////K
					case TOOL_AS_XYZ_EULER_ZY:
						for (j=0; j<6; j++)
							ecp_command.instruction.rmodel.tool_coordinate_def.tool_coordinates[j]
									= EDP_data.next_XYZ_ZYZ_tool_coordinates[j];
						break;
						/////////////////K
					case ARM_KINEMATIC_MODEL:
						ecp_command.instruction.rmodel.kinematic_model.kinematic_model_no
								= EDP_data.next_kinematic_model_no;
						break;
					case SERVO_ALGORITHM:
						for (j=0; j<IRP6_POSTUMENT_NUM_OF_SERVOS; j++) {
							ecp_command.instruction.rmodel.servo_algorithm.servo_algorithm_no[j]
									= EDP_data.next_servo_algorithm_no[j];
							ecp_command.instruction.rmodel.servo_algorithm.servo_parameters_no[j]
									= EDP_data.next_servo_parameters_no[j];
						}
						; // end: for
						break;
					case FORCE_TOOL:
						for (int j=0; j<3; j++) {
							ecp_command.instruction.rmodel.force_tool.position[j]
									= EDP_data.next_force_tool_position[j];
						}
						ecp_command.instruction.rmodel.force_tool.weight
								= EDP_data.next_force_tool_weight;
						break;
					case FORCE_BIAS:
						break;
					default: // Blad: niewlasciwy typ modelu robota
						throw ECP_error(NON_FATAL_ERROR, INVALID_RMODEL_TYPE);
				} // end: switch (set_rmodel_type)
			}

			if (EDP_data.set_type & ARM_DV) {
				ecp_command.instruction.motion_type = EDP_data.motion_type;
				ecp_command.instruction.motion_steps = EDP_data.motion_steps;
				ecp_command.instruction.value_in_step_no = EDP_data.value_in_step_no;
				// Wypelniamy czesc zwiazana z polozeniem ramienia
				switch (EDP_data.set_arm_type) {
					case FRAME:
						copy_frame(ecp_command.instruction.arm.pf_def.arm_frame, EDP_data.next_arm_frame);

						break;
					case XYZ_ANGLE_AXIS:
						for (j=0; j<6; j++)
							ecp_command.instruction.arm.pf_def.arm_coordinates[j]
									= EDP_data.next_XYZ_AA_arm_coordinates[j];

						break;
					case XYZ_EULER_ZYZ:
						for (j=0; j<6; j++)
							ecp_command.instruction.arm.pf_def.arm_coordinates[j]
									= EDP_data.next_XYZ_ZYZ_arm_coordinates[j];

						break;

					case JOINT:
						for (j=0; j<IRP6_POSTUMENT_NUM_OF_SERVOS; j++) {
							ecp_command.instruction.arm.pf_def.desired_torque[j]
									= EDP_data.desired_torque[j];
							ecp_command.instruction.arm.pf_def.arm_coordinates[j]
									= EDP_data.next_joint_arm_coordinates[j];
						}
						break;
					case MOTOR:
						for (j=0; j<IRP6_POSTUMENT_NUM_OF_SERVOS; j++)
							ecp_command.instruction.arm.pf_def.arm_coordinates[j]
									= EDP_data.next_motor_arm_coordinates[j];
						break;
					case PF_VELOCITY:

						for (int j=0; j<IRP6_POSTUMENT_NUM_OF_SERVOS; j++) {
							ecp_command.instruction.arm.pf_def.arm_coordinates[j]
									= EDP_data.next_velocity[j]; // pozycja poczatkowa
						}
						break;
					default: // Blad: niewlasciwy sposob zadawania polozenia ramienia
						throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
				} // end: (set_arm_type)

				switch (EDP_data.next_interpolation_type) {
					case MIM:
						break;
					case TCIM:
						for (int j=0; j<6; j++) {
							ecp_command.instruction.arm.pf_def.inertia[j] = EDP_data.next_inertia[j];
							ecp_command.instruction.arm.pf_def.reciprocal_damping[j]
									= EDP_data.next_reciprocal_damping[j];
							ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[j]
									= EDP_data.next_force_xyz_torque_xyz[j]; // pozycja docelowa
							ecp_command.instruction.arm.pf_def.behaviour[j]
									= EDP_data.next_behaviour[j]; // pozycja docelowa
						}
						break;
					default:
						throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
						break;
				}
				ecp_command.instruction.arm.pf_def.gripper_coordinate
						= EDP_data.next_gripper_coordinate; // zadany stopien rozwarcia chwytaka

			}
			break;
		case GET:
		case SYNCHRO:
		case QUERY:
			break;
		default: // blad: nieprawidlowe polecenie
			throw ECP_error (NON_FATAL_ERROR, INVALID_ECP_COMMAND);
	} // end: switch (instruction_type)

}
; // end: ecp_irp6_postument_robot::create_command
// ---------------------------------------------------------------


/*---------------------------------------------------------------------*/
void ecp_irp6_postument_robot::get_reply(void)
{
	// pobiera z pakietu przeslanego z EDP informacje i wstawia je do
	// odpowiednich skladowych generatora lub warunku

	EDP_data.reply_type = reply_package.reply_type;

	switch (EDP_data.reply_type) {
		case ERROR:
			EDP_data.error_no.error0 = reply_package.error_no.error0;
			EDP_data.error_no.error1 = reply_package.error_no.error1;
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
			throw ECP_error (NON_FATAL_ERROR, INVALID_EDP_REPLY);
	}
}

void ecp_irp6_postument_robot::get_input_reply(void)
{
	EDP_data.input_values = reply_package.input_values;
	for (int i=0; i<8; i++) {
		EDP_data.analog_input[i] =reply_package.analog_input[i];
	}
}

void ecp_irp6_postument_robot::get_arm_reply(void)
{

	switch (reply_package.arm_type) {
		case MOTOR:
			for (int i=0; i<IRP6_POSTUMENT_NUM_OF_SERVOS; i++)
				EDP_data.current_motor_arm_coordinates[i]
						= reply_package.arm.pf_def.arm_coordinates[i];
			break;
		case JOINT:
			for (int i=0; i<IRP6_POSTUMENT_NUM_OF_SERVOS; i++)
				EDP_data.current_joint_arm_coordinates[i]
						= reply_package.arm.pf_def.arm_coordinates[i];
			break;
		case FRAME:
			copy_frame(EDP_data.current_arm_frame, reply_package.arm.pf_def.arm_frame);
			break;
		case XYZ_EULER_ZYZ:
			for (int i=0; i<6; i++)
				EDP_data.current_XYZ_ZYZ_arm_coordinates[i]
						= reply_package.arm.pf_def.arm_coordinates[i];
			break;

		case XYZ_ANGLE_AXIS:
			for (int i=0; i<6; i++)
				EDP_data.current_XYZ_AA_arm_coordinates[i]
						= reply_package.arm.pf_def.arm_coordinates[i];

			break;

		default: // bledny typ specyfikacji pozycji
			throw ECP_error (NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch (...arm_type)


	for (int i=0; i<6; i++) {
		EDP_data.current_force_xyz_torque_xyz[i]
				= reply_package.arm.pf_def.force_xyz_torque_xyz[i];
	}

	EDP_data.current_gripper_coordinate = reply_package.arm.pf_def.gripper_coordinate;
	EDP_data.gripper_reg_state = reply_package.arm.pf_def.gripper_reg_state;
}

void ecp_irp6_postument_robot::get_rmodel_reply(void)
{
	switch (reply_package.rmodel_type) {
		case TOOL_FRAME:
			copy_frame(EDP_data.current_tool_frame, reply_package.rmodel.tool_frame_def.tool_frame);
			break;
		case TOOL_XYZ_ANGLE_AXIS:
			for (int i=0; i<6; i++)
				EDP_data.current_XYZ_AA_tool_coordinates[i]
						= reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			break;
		case TOOL_XYZ_EULER_ZYZ:
			for (int i=0; i<6; i++)
				EDP_data.current_XYZ_ZYZ_tool_coordinates[i]
						= reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			break;
		case TOOL_AS_XYZ_EULER_ZY:
			for (int i=0; i<6; i++)
				EDP_data.current_XYZ_ZYZ_tool_coordinates[i]
						= reply_package.rmodel.tool_coordinate_def.tool_coordinates[i];
			break;
		case ARM_KINEMATIC_MODEL:
			EDP_data.current_kinematic_model_no
					= reply_package.rmodel.kinematic_model.kinematic_model_no;
			break;
		case SERVO_ALGORITHM:
			for (int i=0; i<IRP6_POSTUMENT_NUM_OF_SERVOS; i++) {
				EDP_data.current_servo_algorithm_no[i]
						= reply_package.rmodel.servo_algorithm.servo_algorithm_no[i];
				EDP_data.current_servo_parameters_no[i]
						= reply_package.rmodel.servo_algorithm.servo_parameters_no[i];
			}
			break;
		case FORCE_TOOL:
			for (int j=0; j<3; j++) {
				EDP_data.current_force_tool_position[j]
						= reply_package.rmodel.force_tool.position[j];
			}
			EDP_data.current_force_tool_weight = reply_package.rmodel.force_tool.weight;
			break;
		default: // bledny typ specyfikacji modelu robota
			throw ECP_error(NON_FATAL_ERROR, INVALID_POSE_SPECIFICATION);
	} // end: switch (...rmodel_type)
}

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp
