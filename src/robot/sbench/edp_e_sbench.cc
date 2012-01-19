/*!
 * @file
 * @brief File containing the definition of edp::sbench::effector class.
 *
 * @author yoyek
 *
 * @ingroup sbench
 */

#include <cstdio>

#include <comedilib.h>

#include "robot/canopen/gateway_epos_usb.h"
#include "robot/canopen/gateway_socketcan.h"

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

#include "edp_e_sbench.h"
#include "base/edp/reader.h"

#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace sbench {

#include "base/lib/debug.hpp"

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::single_thread_master_order(nm_task, nm_tryb);
}

effector::effector(common::shell &_shell) :
		motor_driven_effector(_shell, lib::sbench::ROBOT_NAME, instruction, reply),
		dev_name("/dev/comedi0")
{
	DEBUG_METHOD;

	if (robot_test_mode) {
		// If robot test mode - deactivate both flags modes automatically.
		power_supply = 0;
		cleaning  = 0;
	} else {
		// Read values from config file.
		power_supply = config.exists_and_true("power_supply");
		cleaning = config.exists_and_true("cleaning");
	}



	// TODO: what for??
	number_of_servos = lib::sbench::NUM_OF_SERVOS;

	// Initialize power and pressure supplies.
	power_supply_init();
	cleaning_init();
}



effector::~effector()
{
	DEBUG_METHOD;
	if(power_supply_active()) {
		// Detach from hardware
		if (power_supply_device) {
			if(comedi_close(power_supply_device) == -1) {
				throw std::runtime_error("Could not close the power supply device.");
			}
		}
	}
}


bool effector::cleaning_active()
{
	return cleaning;
}

bool effector::power_supply_active()
{
	return power_supply;
}


void effector::power_supply_init()
{
	DEBUG_METHOD;
	if (!power_supply_active()) {
		msg->message("Power supply of relays will not used - test mode activated");
		// NULL pointer just for safety.
		power_supply_device = NULL;
		current_pins_buf.voltage_buf.set_all_off();
	} else {
		// Initialize the hardware controlling the power supply.
		power_supply_device = comedi_open(dev_name.c_str());
		if (!power_supply_device) {
			throw std::runtime_error("Could not open the power supply device.");
		}
	}
}


void effector::cleaning_init()
{
	DEBUG_METHOD;

	// Inform the user about the configuration.
	if (!cleaning_active()) {
		msg->message("FESTO hardware will not used for cleaning - test mode activated");
		current_pins_buf.preasure_buf.set_all_off();
	} else {
		// Initialize the can connection.
		gateway = (boost::shared_ptr <canopen::gateway>) new canopen::gateway_epos_usb();

		// Connect to the gateway.
		gateway->open();

		// Create festo node.
		cpv10 = (boost::shared_ptr <festo::cpv>) new festo::cpv(*gateway, FESTO_ADRESS);

		festo::U32 DeviceType = cpv10->getDeviceType();
		printf("Device type = 0x%08X\n", DeviceType);

		festo::U8 ErrorRegister = cpv10->getErrorRegister();
		printf("Error register = 0x%02X\n", ErrorRegister);

		festo::U32 ManufacturerStatusRegister = cpv10->getManufacturerStatusRegister();
		printf("Manufacturer status register = 0x%08X\n", ManufacturerStatusRegister);

		festo::U8 NumberOfErrorsInDiagnosticMemeory = cpv10->getNumberOfErrorsInDiagnosticMemeory();
		printf("Number of errors in diagnostic memory = %d\n", NumberOfErrorsInDiagnosticMemeory);
		if (NumberOfErrorsInDiagnosticMemeory > 0) {
			cpv10->clearErrorsInDiagnosticMemeory();
		}

		printf("Status byte = 0x%02x\n", cpv10->getStatusByte());

		uint8_t NumberOfOutputGroups = cpv10->getNumberOf8OutputGroups();
		printf("Number of 8-output groups = %d\n", NumberOfOutputGroups);

		uint8_t Outputs07 = cpv10->getOutputs(1);
		printf("Status of outputs 0..7 = 0x%02x\n", Outputs07);

		gateway->SendNMTService(FESTO_ADRESS, canopen::gateway::Start_Remote_Node);
		//gateway->SendNMTService(FESTO_ADRESS, canopen::gateway::Reset_Node);

		current_pins_buf.preasure_buf.set_all_off();
	}
}

void effector::get_controller_state(lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	controller_state_edp_buf.is_synchronised = true;
	reply.controller_state = controller_state_edp_buf;
}

void effector::move_arm(const lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	switch (instruction.sbench.variant)
	{
		case lib::sbench::POWER_SUPPLY:
			DEBUG_COMMAND("VOLTAGE");
			power_supply_command();
			break;
		case lib::sbench::CLEANING:
			DEBUG_COMMAND("PREASURE");
			cleaning_command();
			break;
	}

}

void effector::power_supply_command()
{
	DEBUG_METHOD;

	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	lib::sbench::power_supply_state voltage_buf = instruction.sbench.voltage_buf;

	// Check working mode.
	if (!power_supply_active()) {

		for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {
			if (voltage_buf.pins_state[i]) {
				ss << "1";
			} else {
				ss << "0";
			}
		}
		current_pins_buf.voltage_buf = voltage_buf;
		ss << std::endl;
		msg->message(ss.str());
	} else {
		for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {
			if (voltage_buf.pins_state[i]) {
				ss << "1";
			} else {
				ss << "0";
			}
		}
		current_pins_buf.voltage_buf = voltage_buf;
		ss << std::endl;
		msg->message(ss.str());

		int total_number_of_pins_activated = 0;
		for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {
			if (voltage_buf.pins_state[i]) {
				total_number_of_pins_activated++;
			}
		}

		if (total_number_of_pins_activated <= VOLTAGE_PINS_ACTIVATED_LIMIT) {
			for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {
				comedi_dio_write(power_supply_device, (unsigned int) (i / 32), (unsigned int) (i % 32), (unsigned int) voltage_buf.pins_state[i]);
				//	comedi_dio_write(voltage_device, (int) (i / 32), (i % 32), 0);
			} // send command to hardware
		} else {
			// TODO throw
			msg->message(lib::NON_FATAL_ERROR, "voltage_command total_number_of_pins_activated exceeded");
		}

	}

}

void effector::cleaning_command()
{
	DEBUG_METHOD;

	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	lib::sbench::cleaning_state preasure_buf = instruction.sbench.preasure_buf;

	// Check working mode.
	if (!cleaning_active()) {
		for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {
			if (preasure_buf.pins_state[i]) {
				ss << "1";
			} else {
				ss << "0";
			}
		}

		ss << std::endl;
		msg->message(ss.str());
		current_pins_buf.preasure_buf = preasure_buf;
	} else {
		msg->message("preasure_command hardware mode");

		for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {
			if (preasure_buf.pins_state[i]) {
				ss << "1";
			} else {
				ss << "0";
			}
		}

		ss << std::endl;
		msg->message(ss.str());

		int total_number_of_pins_activated = 0;

		// prepare the desired output
		for (int i = 0; i < NUMBER_OF_FESTO_GROUPS; i++) {
			for (int j = 0; j < 8; j++) {
				desired_output[i + 1][j] = preasure_buf.pins_state[i * 8 + j];
				if (preasure_buf.pins_state[i * 8 + j]) {
					total_number_of_pins_activated++;
				}
			}
		}

		// checks if the limit was exceded
		if (total_number_of_pins_activated <= CLEANING_PINS_ACTIVATED_LIMIT) {
			for (int i = 0; i < NUMBER_OF_FESTO_GROUPS; i++) {
				cpv10->setOutputs(i + 1, (uint8_t) desired_output[i + 1].to_ulong());
			}
		} else {
			// TODO throw
			msg->message(lib::NON_FATAL_ERROR, "preasure_command total_number_of_pins_activated exceeded");
		}

//		for (int k = 0; k < 20; k++) {
//			// send the command
//			for (int i = 0; i < NUMBER_OF_FESTO_GROUPS; i++) {
//				//	cpv10->setOutputs(i + 1, (uint8_t) desired_output[i + 1].to_ulong());
//				cpv10->setOutputs(i + 1, (uint8_t) 0xAA);
//			}delay(1000);
//
//			// send the command
//			for (int i = 0; i < NUMBER_OF_FESTO_GROUPS; i++) {
//				//	cpv10->setOutputs(i + 1, (uint8_t) desired_output[i + 1].to_ulong());
//				cpv10->setOutputs(i + 1, (uint8_t) 0x55);
//			}delay(1000);
//
//			// send the command
//			for (int i = 0; i < NUMBER_OF_FESTO_GROUPS; i++) {
//				//	cpv10->setOutputs(i + 1, (uint8_t) desired_output[i + 1].to_ulong());
//				cpv10->setOutputs(i + 1, (uint8_t) 0x00);
//			}
//			delay(1000);
//		}

		//current_pins_buf.preasure_buf = preasure_buf;

	}

}

void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction_)
{
	DEBUG_METHOD;

	power_supply_reply();
	cleaning_reply();
	reply.sbench = current_pins_buf;
	reply.servo_step = step_counter;
}

void effector::power_supply_reply()
{
	DEBUG_METHOD;

	DEBUG_COMMAND("VOLTAGE");
	if (power_supply_active()) {
		// read pin_state from hardware
		for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {
			unsigned int current_read;
			comedi_dio_read(power_supply_device, (int) (i / 32), (i % 32), &current_read);
			current_pins_buf.voltage_buf.pins_state[i] = current_read;
		} // send command to hardware
	}
}

void effector::cleaning_reply()
{
	DEBUG_METHOD;

	DEBUG_COMMAND("PREASURE");
	if (cleaning_active()) {
		for (int i = 0; i < NUMBER_OF_FESTO_GROUPS; i++) {
			current_output[i + 1] = cpv10->getOutputs(i + 1);
		}

		for (int i = 0; i < NUMBER_OF_FESTO_GROUPS; i++) {
			for (int j = 0; j < 8; j++) {
				current_pins_buf.preasure_buf.pins_state[i * 8 + j] = current_output[i + 1][j];
			}
		}

	}
}

void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
}

lib::INSTRUCTION_TYPE effector::variant_receive_instruction()
{
	return receive_instruction(instruction);
}

void effector::variant_reply_to_instruction()
{
	reply_to_instruction(reply);
}

void effector::create_kinematic_models_for_given_robot(void)
{
	// There are no kinematic models.
}


} // namespace sbench

namespace common {

effector* return_created_efector(common::shell &_shell)
{
	return new sbench::effector(_shell);
}
}

} // namespace edp
} // namespace mrrocpp

