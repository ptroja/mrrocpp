#include <cstdio>
#include <boost/static_assert.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/mrmath/mrmath.h"

// Klasa edp_irp6ot_effector.
#include "edp_e_sbench.h"
#include "base/edp/reader.h"
// Kinematyki.
#include "robot/sbench/kinematic_model_sbench.h"
#include "base/edp/manip_trans_t.h"
#include "base/edp/vis_server.h"

#include "base/lib/exception.h"
using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace sbench {

void effector::master_order(common::MT_ORDER nm_task, int nm_tryb)
{
	motor_driven_effector::single_thread_master_order(nm_task, nm_tryb);
}

// Konstruktor.
effector::effector(common::shell &_shell) :
		motor_driven_effector(_shell, lib::sbench::ROBOT_NAME, instruction, reply), dev_name("/dev/comedi0")
{

	number_of_servos = lib::sbench::NUM_OF_SERVOS;
	//  Stworzenie listy dostepnych kinematyk.
	create_kinematic_models_for_given_robot();

	reset_variables();
	voltage_init();
	preasure_init();
}

void effector::voltage_init()
{
	if (!robot_test_mode) {

		// initiate hardware
		voltage_device = comedi_open(dev_name.c_str());

		if (!voltage_device) {

			throw std::runtime_error("Could not open voltage_device");
		}

	} else {
		// Set pointer just for safety
		device = NULL;

		current_pins_buf.voltage_buf.set_zeros();
	}
}

effector::~effector()
{
	if(!robot_test_mode) {
		// Detach from hardware
		if (device) {
			if(comedi_close(device) == -1) {
				throw std::runtime_error("comedi_close() failed");
			}
		}
	}
}

void effector::preasure_init()
{
	if (!robot_test_mode) {

	} else {

		current_pins_buf.preasure_buf.set_zeros();
	}
}

void effector::get_controller_state(lib::c_buffer &instruction)
{
	controller_state_edp_buf.is_synchronised = true;
	reply.controller_state = controller_state_edp_buf;
}

/*--------------------------------------------------------------------------*/
void effector::move_arm(const lib::c_buffer &instruction)
{

	const lib::sbench::c_buffer & local_instruction = (lib::sbench::c_buffer &) instruction;

	msg->message("move_arm");

	switch (local_instruction.sbench.variant)
	{

		case lib::sbench::VOLTAGE:
			voltage_command(local_instruction);
			break;
		case lib::sbench::PREASURE:
			preasure_command(local_instruction);
			break;

	}

}

void effector::voltage_command(lib::sbench::c_buffer &instruction)
{
	msg->message("voltage_command");
	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	lib::sbench::voltage_buffer voltage_buf = instruction.sbench.voltage_buf;

	if (robot_test_mode) {

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
			comedi_dio_write(voltage_device, (int) (i / 32), (i % 32), voltage_buf.pins_state[i]);
		} // send command to hardware
	}

}

void effector::preasure_command(lib::sbench::c_buffer &instruction)
{
	msg->message("preasure_command");
}

/*--------------------------------------------------------------------------*/
void effector::get_arm_position(bool read_hardware, lib::c_buffer &instruction)
{
	msg->message("get_arm");

	//lib::JointArray desired_joints_tmp(lib::MAX_SERVOS_NR); // Wspolrzedne wewnetrzne -
	//	printf(" GET ARM\n");
	//	flushall();
	static int licznikaaa = (-11);

	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	ss << "get_arm_position: " << licznikaaa;
	msg->message(ss.str().c_str());
	//	printf("%s\n", ss.str().c_str());

	voltage_reply();
	preasure_reply();

	reply.servo_step = step_counter;
}
/*--------------------------------------------------------------------------*/

void effector::voltage_reply()
{
	if (!robot_test_mode) {

		// read pin_state from hardware

		for (int i = 0; i < lib::sbench::NUM_OF_PINS; i++) {
			unsigned int current_read;
			comedi_dio_read(voltage_device, (int) (i / 32), (i % 32), &current_read);
			current_pins_buf.voltage_buf.pins_state[i] = current_read;
		} // send command to hardware

	}
	reply.sbench.voltage_buf = current_pins_buf.voltage_buf;
}

void effector::preasure_reply()
{

}

// Stworzenie modeli kinematyki dla robota IRp-6 na postumencie.
void effector::create_kinematic_models_for_given_robot(void)
{
	// Stworzenie wszystkich modeli kinematyki.
	add_kinematic_model(new kinematics::sbench::model());
	// Ustawienie aktywnego modelu.
	set_kinematic_model(0);
}

/*--------------------------------------------------------------------------*/
void effector::create_threads()
{
	rb_obj = (boost::shared_ptr <common::reader_buffer>) new common::reader_buffer(*this);
	//vis_obj = (boost::shared_ptr <common::vis_server>) new common::vis_server(*this);
}

lib::INSTRUCTION_TYPE effector::receive_instruction()
{
	return common::effector::receive_instruction(instruction);
}

void effector::variant_reply_to_instruction()
{
	reply_to_instruction(reply);
}

} // namespace smb

namespace common {

effector* return_created_efector(common::shell &_shell)
{
	return new sbench::effector(_shell);
}
}

} // namespace edp
} // namespace mrrocpp

