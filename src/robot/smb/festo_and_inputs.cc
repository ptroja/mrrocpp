/*!
 * \brief File containing the festo and inputs class methods
 * it controls the pneumatic legs in edp smb
 *
 * \author yoyek
 * \date 2011
 *
 */

#include "base/edp/edp_e_motor_driven.h"
#include "const_smb.h"
#include "edp_e_smb.h"
#include "../canopen/gateway_epos_usb.h"
#include "../canopen/gateway_socketcan.h"
#include "../festo/cpv.h"
#include "../maxon/epos.h"
#include "festo_and_inputs.h"
#include "exceptions.h"

namespace mrrocpp {
namespace edp {
namespace smb {

festo_and_inputs::festo_and_inputs(effector &_master) :
		master(_master), epos_di_node(master.legs_rotation_node), cpv10(master.cpv10), robot_test_mode(master.robot_test_mode)
{
	if (!(robot_test_mode)) {
		// prepares hardware
		festo::U32 DeviceType = cpv10->getDeviceType();
		printf("Device type = 0x%08X\n", DeviceType);

		festo::U8 ErrorRegister = cpv10->getErrorRegister();
		printf("Error register = 0x%02X\n", ErrorRegister);

		festo::U32 ManufacturerStatusRegister = cpv10->getManufacturerStatusRegister();
		printf("Manufacturer status register = 0x%08X\n", ManufacturerStatusRegister);

		uint8_t NumberOfOutputGroups = cpv10->getNumberOf8OutputGroups();
		printf("Number of 8-output groups = %d\n", NumberOfOutputGroups);

		uint8_t Outputs07 = cpv10->getOutputs(1);
		printf("Status of outputs 0..7 = 0x%02x\n", Outputs07);

		master.gateway->SendNMTService(10, canopen::gateway::Start_Remote_Node);

		determine_legs_state();
		desired_output[1] = current_output[1];
		desired_output[2] = current_output[2];

	} else {
		current_legs_state = lib::smb::ALL_UP;
		for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
			master.edp_ecp_rbuffer.multi_leg_reply.leg[i].is_up = true;
			master.edp_ecp_rbuffer.multi_leg_reply.leg[i].is_down = false;
		}

	}

	next_legs_state = current_legs_state;
}

festo_and_inputs::~festo_and_inputs()
{

}

bool festo_and_inputs::is_checked(int leg_number)
{
	return checked[leg_number - 1];
}

void festo_and_inputs::set_checked(int leg_number)
{
	checked[leg_number - 1] = true;
}

void festo_and_inputs::set_unchecked(int leg_number)
{
	checked[leg_number - 1] = false;
}

void festo_and_inputs::set_all_legs_unchecked()
{
	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		checked[i] = false;
	}
}

bool festo_and_inputs::is_upper_halotron_active(int leg_number)
{
	return epos_inputs[2 * leg_number + 9];
}

bool festo_and_inputs::is_lower_halotron_active(int leg_number)
{
	return epos_inputs[2 * leg_number + 8];
}

bool festo_and_inputs::is_attached(int leg_number)
{
	// to be implemented when the appropriate hardware will be installed
	return false;
}

void festo_and_inputs::set_detach(int leg_number, bool value)
{
	// undetachable legs can not be detached !!
	if ((value == true) && (festo_command.undetachable[leg_number - 1] == true)) {
		return;
	}

	switch (leg_number)
	{
		case 1: {
			desired_output[FESTO_C1_GROUP][FESTO_C1_BIT_TO_SET] = value;
		}
			break;
		case 2: {
			desired_output[FESTO_C2_GROUP][FESTO_C2_BIT_TO_SET] = value;
		}

			break;
		case 3: {
			desired_output[FESTO_C3_GROUP][FESTO_C3_BIT_TO_SET] = value;
		}
			break;

		default:
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(INVALID_MOTION_PARAMETERS));
			break;
	}
}

void festo_and_inputs::set_move_up(int leg_number, bool value)
{
	switch (leg_number)
	{
		case 1: {
			desired_output[FESTO_CY11_GROUP][FESTO_CY11_BIT_TO_SET] = value;
		}

			break;
		case 2: {
			desired_output[FESTO_CY21_GROUP][FESTO_CY21_BIT_TO_SET] = value;
		}

			break;
		case 3: {
			desired_output[FESTO_CY31_GROUP][FESTO_CY31_BIT_TO_SET] = value;
		}

			break;

		default:
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(INVALID_MOTION_PARAMETERS));
			break;

	}
}

void festo_and_inputs::set_move_down(int leg_number, bool value)
{
	switch (leg_number)
	{
		case 1: {
			desired_output[FESTO_CY12_GROUP][FESTO_CY12_BIT_TO_SET] = value;
		}

			break;
		case 2: {
			desired_output[FESTO_CY22_GROUP][FESTO_CY22_BIT_TO_SET] = value;
		}

			break;
		case 3: {
			desired_output[FESTO_CY32_GROUP][FESTO_CY32_BIT_TO_SET] = value;
		}

			break;

		default:
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(INVALID_MOTION_PARAMETERS));
			break;

	}
}

void festo_and_inputs::set_clean(int leg_number, bool value)
{
	switch (leg_number)
	{
		case 1: {
			desired_output[FESTO_CH1_GROUP][FESTO_CH1_BIT_TO_SET] = value;
		}
			break;
		case 2: {
			desired_output[FESTO_CH2_GROUP][FESTO_CH2_BIT_TO_SET] = value;
		}
			break;
		case 3: {
			desired_output[FESTO_CH3_GROUP][FESTO_CH3_BIT_TO_SET] = value;
		}
			break;

		default:
			BOOST_THROW_EXCEPTION(nfe_2() << mrrocpp_error0(INVALID_MOTION_PARAMETERS));
			break;

	}
}

void festo_and_inputs::determine_legs_state()
{
	if (!(robot_test_mode)) {
		read_state();
		int number_of_legs_up = 0;

		for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {

			if (is_upper_halotron_active(i + 1)) {
				number_of_legs_up++;
			}
		}

		switch (number_of_legs_up)
		{
			case 0:

				current_legs_state = lib::smb::ALL_DOWN;
				break;
			case 1:
				current_legs_state = lib::smb::ONE_UP_TWO_DOWN;
				break;
			case 2:
				current_legs_state = lib::smb::TWO_UP_ONE_DOWN;
				break;
			case 3:
				current_legs_state = lib::smb::ALL_UP;
				break;
			default:
				break;

		}

	}
}

/*--------------------------------------------------------------------------*/

void festo_and_inputs::command()
{
	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	master.msg->message("FESTO");

	memcpy(&festo_command, &(master.ecp_edp_cbuffer.festo_command), sizeof(festo_command));

	if (robot_test_mode) {
		ss << festo_command.leg[2];

		master.msg->message(ss.str().c_str());

	} else {
		determine_legs_state();
	}

	// determine next_legs_state by counting numebr of legs to be up
	int number_of_legs_up = 0;
	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		if (festo_command.leg[i] == lib::smb::UP) {
			number_of_legs_up++;
		}

	}

	switch (number_of_legs_up)
	{
		case 0:
			next_legs_state = lib::smb::ALL_DOWN;
			break;
		case 1:
			next_legs_state = lib::smb::ONE_UP_TWO_DOWN;
			break;
		case 2:
			next_legs_state = lib::smb::TWO_UP_ONE_DOWN;
			break;
		case 3:
			next_legs_state = lib::smb::ALL_UP;
			break;
		default:
			break;

	}

	// checks if the next_legs_state is valid taking into account current_legs_state
	// and prepares detailed command for festo hardware

	switch (next_legs_state)
	{
		case lib::smb::ALL_DOWN:
			command_all_down();
			break;
		case lib::smb::ONE_UP_TWO_DOWN:
			command_one_up_two_down();
			break;
		case lib::smb::TWO_UP_ONE_DOWN:
			command_two_up_one_down();
			break;
		case lib::smb::ALL_UP:
			command_all_up();
			break;
		default:
			break;

	}

	determine_legs_state();

	if (robot_test_mode) {
		// the previous next_legs_state becomes currrent_state
		current_legs_state = next_legs_state;
	}

}

void festo_and_inputs::move_one_or_two_down()
{
	// detach all legs that are up to prepare them to go down

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {

		if (!is_lower_halotron_active(i + 1)) {

			set_detach(i + 1, true);
		}
		// for safety reasons
		if (is_lower_halotron_active(i + 1)) {

			set_detach(i + 1, false);
		}

	}

	execute_command();
	delay(500);

	// move the legs down
	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {

		set_move_down(i + 1, true);
		set_move_up(i + 1, false);

	}

	execute_command();

	// waits until all legs go down

	int number_of_legs_down = 0;

	set_all_legs_unchecked();

	for (int iteration = 0; number_of_legs_down < 3; iteration++) {

		//	master.msg->message("wait iteration");
		delay(FAI_SINGLE_DELAY);

		// if it take too long to wait break

		if (iteration > FAI_DELAY_MAX_ITERATION) {
			master.msg->message(lib::NON_FATAL_ERROR, "LEGS MOTION WAIT TIMEOUT");

			break;
		}

		read_state();
		for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {

			if ((!is_checked(i + 1)) && (is_lower_halotron_active(i + 1))) {
				set_checked(i + 1);

				number_of_legs_down++;
				// attach leg
				set_detach(i + 1, false);

			}

		}
	}

	// wait a while in case the legs are still in motion
	delay(500);

	// attach legs
	execute_command();
}

void festo_and_inputs::command_all_down()
{
	master.msg->message("command_all_down");
	switch (current_legs_state)
	{
		case lib::smb::ALL_DOWN: {
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_invalid_command_in_given_state() << current_state(current_legs_state) << retrieved_festo_command(lib::smb::ALL_DOWN));
		}
			break;
		case lib::smb::ONE_UP_TWO_DOWN: {
			master.msg->message("ONE_UP_TWO_DOWN");
			if (!test_mode_set_reply()) {
				move_one_or_two_down();
			}

		}
			break;
		case lib::smb::TWO_UP_ONE_DOWN: {
			master.msg->message("TWO_UP_ONE_DOWN");
			if (!test_mode_set_reply()) {
				move_one_or_two_down();
			}

		}
			break;
		case lib::smb::ALL_UP: {
			master.msg->message("ALL_UP");
			if (!test_mode_set_reply()) {

				// moves all legs down and does not detach them !

				for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
					set_move_down(i + 1, true);
					set_move_up(i + 1, false);
					set_detach(i + 1, false);
				}

				execute_command();

				// waits until all legs are in down position
				int number_of_legs_down = 0;

				set_all_legs_unchecked();

				for (int iteration = 0; number_of_legs_down < 3; iteration++) {

					delay(FAI_SINGLE_DELAY);

					// if it take too long to wait break
					if (iteration > FAI_DELAY_MAX_ITERATION) {
						master.msg->message(lib::NON_FATAL_ERROR, "LEGS MOTION WAIT TIMEOUT");

						break;
					}

					read_state();

					for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {

						if ((!is_checked(i + 1)) && (is_lower_halotron_active(i + 1))) {
							set_checked(i + 1);
							number_of_legs_down++;
						}

					}
				}
			}
		}
			break;
		default:
			break;

	}
}

void festo_and_inputs::command_one_up_two_down()
{
	BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_invalid_command_in_given_state()<<current_state(current_legs_state) << retrieved_festo_command(lib::smb::ONE_UP_TWO_DOWN));
}

void festo_and_inputs::command_two_up_one_down()
{

#ifdef COMMAND_TWO_UP_ONE_DOWN_SIMULTANEOUS
	// HIGH PRESSURE VERSION WITH SIMULATNOUS LEG MOTION
	switch(current_legs_state) {
		case lib::smb::ALL_DOWN: {
			if (!test_mode_set_reply()) {

				// detaches the leg that are to move up

				for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
					if (festo_command.leg[i] == lib::smb::UP) {
						if (is_lower_halotron_active(i + 1)) {
							set_detach(i + 1, true);
							set_move_down(i + 1, false);
						}
					}
				}
				execute_command();

				//waits a while for lockers to move
				delay(500);

				// move equivalent legs up
				for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
					if (festo_command.leg[i] == lib::smb::UP) {
						if (is_lower_halotron_active(i + 1)) {
							set_move_up(i + 1, true);
						}
					}
				}
				execute_command();

				// wait for legs to be in upper position
				int number_of_legs_up = 0;

				set_all_legs_unchecked();

				for (int iteration = 0; number_of_legs_up < 2; iteration++) {
					delay(FAI_SINGLE_DELAY);

					// if it take too long to wait break
					if (iteration > FAI_DELAY_MAX_ITERATION) {
						master.msg->message(lib::NON_FATAL_ERROR, "LEGS MOTION WAIT TIMEOUT");

						break;
					}

					read_state();
					for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
						if ((!is_checked(i + 1)) && (is_upper_halotron_active(i + 1))
								&& (festo_command.leg[i] == lib::smb::UP)) {
							set_checked(i + 1);
							number_of_legs_up++;
							set_detach(i + 1, false);
						}
					}
				}

				execute_command();
			}
		}

		break;
		case lib::smb::ONE_UP_TWO_DOWN:
		case lib::smb::TWO_UP_ONE_DOWN:
		case lib::smb::ALL_UP: {
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_invalid_command_in_given_state()<<current_state(current_legs_state) << retrieved_festo_command(lib::smb::TWO_UP_ONE_DOWN));
		}
		break;
		default:
		break;

	}

# else
	// LOW PRESSURE VERSION WITH SERIAL LEG MOTION
	switch (current_legs_state)
	{
		case lib::smb::ALL_DOWN: {
			if (!test_mode_set_reply()) {

				// move the legs in series

				for (int number_of_legs_moved = 0; number_of_legs_moved < 2; number_of_legs_moved++) {

					// detaches the leg that are to move up

					for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
						if (festo_command.leg[i] == lib::smb::UP) {
							if (is_lower_halotron_active(i + 1)) {
								set_detach(i + 1, true);
								set_move_down(i + 1, false);
								break;
							}
						}
					}
					execute_command();

					//waits a while for lockers to move
					delay(500);

					for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
						if (festo_command.leg[i] == lib::smb::UP) {
							if (is_lower_halotron_active(i + 1)) {
								set_move_up(i + 1, true);
								break;
							}
						}
					}
					execute_command();

					// wait for legs to be in upper position
					int number_of_legs_up = 0;

					set_all_legs_unchecked();

					for (int iteration = 0; number_of_legs_up < number_of_legs_moved + 1; iteration++) {
						delay(FAI_SINGLE_DELAY);

						// if it takes too long to wait then break.
						if (iteration > FAI_DELAY_MAX_ITERATION) {
							master.msg->message(lib::NON_FATAL_ERROR, "LEGS MOTION WAIT TIMEOUT");

							break;
						}

						read_state();
						for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
							if ((!is_checked(i + 1)) && (is_upper_halotron_active(i + 1))
									&& (festo_command.leg[i] == lib::smb::UP)) {
								set_checked(i + 1);
								number_of_legs_up++;
								set_detach(i + 1, false);
							}
						}
					}

					execute_command();
				}
			}
		}

			break;
		case lib::smb::ONE_UP_TWO_DOWN:
		case lib::smb::TWO_UP_ONE_DOWN:
		case lib::smb::ALL_UP: {
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_invalid_command_in_given_state()<<current_state(current_legs_state) << retrieved_festo_command(lib::smb::TWO_UP_ONE_DOWN));
		}
			break;
		default:
			break;

	}

#endif

}

void festo_and_inputs::command_all_up()
{
	switch (current_legs_state)
	{
		case lib::smb::ALL_DOWN: {
			if (!test_mode_set_reply()) {

				// move all legs up and do not detach them;

				for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
					set_move_down(i + 1, false);
					set_move_up(i + 1, true);
				}

				execute_command();

				// waits until all legs are in upper position

				int number_of_legs_up = 0;

				set_all_legs_unchecked();

				for (int iteration = 0; number_of_legs_up < 3; iteration++) {

					delay(FAI_SINGLE_DELAY);

					// if it take too long to wait break
					if (iteration > FAI_DELAY_MAX_ITERATION) {
						master.msg->message(lib::NON_FATAL_ERROR, "LEGS MOTION WAIT TIMEOUT");

						break;
					}

					read_state();
					for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {

						if ((!is_checked(i + 1)) && (is_upper_halotron_active(i + 1))) {
							set_checked(i + 1);
							number_of_legs_up++;
						}

					}
				}
			}
		}

			break;
		case lib::smb::ONE_UP_TWO_DOWN:
		case lib::smb::TWO_UP_ONE_DOWN:
		case lib::smb::ALL_UP:
			BOOST_THROW_EXCEPTION(mrrocpp::edp::smb::nfe_invalid_command_in_given_state()<<current_state(current_legs_state) << retrieved_festo_command(lib::smb::ALL_UP));
			break;
		default:
			break;

	}
}

bool festo_and_inputs::test_mode_set_reply()
{
	if (robot_test_mode) {
		for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
			if (festo_command.leg[i] == lib::smb::UP) {
				master.edp_ecp_rbuffer.multi_leg_reply.leg[i].is_up = true;
				master.edp_ecp_rbuffer.multi_leg_reply.leg[i].is_down = false;
			} else {
				master.edp_ecp_rbuffer.multi_leg_reply.leg[i].is_up = false;
				master.edp_ecp_rbuffer.multi_leg_reply.leg[i].is_down = true;
			}

		}
	}
	return robot_test_mode;
}

void festo_and_inputs::read_state()
{
	if (!(robot_test_mode)) {
		epos_inputs = epos_di_node->getDInput();

		current_output[1] = cpv10->getOutputs(1);
		current_output[2] = cpv10->getOutputs(2);
	}
}

void festo_and_inputs::create_reply()
{

	determine_legs_state();

	if (!robot_test_mode) {
		read_state();
		for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
			master.edp_ecp_rbuffer.multi_leg_reply.leg[i].is_down = is_lower_halotron_active(i + 1);
			master.edp_ecp_rbuffer.multi_leg_reply.leg[i].is_up = is_upper_halotron_active(i + 1);
			master.edp_ecp_rbuffer.multi_leg_reply.leg[i].is_attached = is_attached(i + 1);
		}
		//std::cout << "epos digital inputs = " << epos_digits << std::endl;
	}
}

void festo_and_inputs::execute_command()
{
	if (!robot_test_mode) {
		cpv10->setOutputs(1, (uint8_t) desired_output[1].to_ulong());
		cpv10->setOutputs(2, (uint8_t) desired_output[2].to_ulong());
		//	std::cout << "desired_output = " << desired_output[2] << std::endl;
		read_state();
		desired_output[1] = current_output[1];
		desired_output[2] = current_output[2];

	}
}

} // namespace smb
} // namespace edp
} // namespace mrrocpp

