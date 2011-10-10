/*!
 * \file festo_and_inputs.cc
 * \brief File containing the festo and inputs class methods
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

namespace mrrocpp {
namespace edp {
namespace smb {

festo_and_inputs::festo_and_inputs(effector &_master, boost::shared_ptr <maxon::epos> _epos_di_node, boost::shared_ptr <
		festo::cpv> _cpv10) :
		master(_master), epos_di_node(_epos_di_node), cpv10(_cpv10)
{
	if (!(master.robot_test_mode)) {
		festo::U32 DeviceType = cpv10->readDeviceType();
		printf("Device type = 0x%08X\n", DeviceType);

		festo::U8 ErrorRegister = cpv10->readErrorRegister();
		printf("Error register = 0x%02X\n", ErrorRegister);

		festo::U32 ManufacturerStatusRegister = cpv10->readManufacturerStatusRegister();
		printf("Manufacturer status register = 0x%08X\n", ManufacturerStatusRegister);

		uint8_t NumberOfOutputGroups = cpv10->readNumberOf8OutputGroups();
		printf("Number of 8-output groups = %d\n", NumberOfOutputGroups);

		uint8_t Outputs07 = cpv10->readOutputs(1);
		printf("Status of outputs 0..7 = 0x%02x\n", Outputs07);

		master.gateway->SendNMTService(10, canopen::gateway::Start_Remote_Node);

		read_state();
	}
}

festo_and_inputs::~festo_and_inputs()
{

}

bool festo_and_inputs::is_upper_halotron_avtive(int leg_number)
{
	return epos_inputs[2 * leg_number + 11];
}

bool festo_and_inputs::is_lower_halotron_avtive(int leg_number)
{
	return epos_inputs[2 * leg_number + 10];
}

bool festo_and_inputs::is_attached(int leg_number)
{
	// to be implemented
	return false;
}

void festo_and_inputs::set_detach(int leg_number, bool value)
{
	switch (leg_number)
	{
		case 1: {
			//	group_one_desired_output[2] = value;
		}

			break;
		case 2: {

		}

			break;
		case 3: {

		}

			break;

		default:
			throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
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
			throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
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
			throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
			break;

	}
}

void festo_and_inputs::set_clean(int leg_number, bool value)
{
	switch (leg_number)
	{
		case 1: {
			//	group_one_desired_output[2] = value;
		}

			break;
		case 2: {

		}

			break;
		case 3: {

		}

			break;

		default:
			throw NonFatal_error_2(INVALID_MOTION_PARAMETERS);
			break;

	}
}

void festo_and_inputs::read_state()
{
	if (!(master.robot_test_mode)) {
		epos_inputs = epos_di_node->readDInput();

		current_output[1] = cpv10->readOutputs(1);
		desired_output[1] = current_output[1];
		//	std::cout << "group_one_desired_output 1= " << group_one_desired_output << std::endl;
		current_output[2] = cpv10->readOutputs(2);
		//	std::cout << "group_two_current_output 1= " << group_two_current_output << std::endl;
		desired_output[2] = current_output[2];
	}
}

void festo_and_inputs::execute_command()
{
	cpv10->writeOutputs(1, (uint8_t) desired_output[1].to_ulong());
	cpv10->writeOutputs(2, (uint8_t) desired_output[2].to_ulong());
}

} // namespace smb
} // namespace edp
} // namespace mrrocpp

