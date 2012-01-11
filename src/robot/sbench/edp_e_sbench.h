/*!
 * \file edp_e_sbench.h
 * \brief File containing the declaration of edp::sbench::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */

#ifndef __EDP_E_SBENCH_H
#define __EDP_E_SBENCH_H

#include "base/edp/edp_e_motor_driven.h"
#include "dp_sbench.h"
#include <comedilib.h>
#include "../canopen/gateway_epos_usb.h"
#include "../canopen/gateway_socketcan.h"
#include "../festo/cpv.h"
#include "../maxon/epos.h"

namespace mrrocpp {
namespace edp {
namespace sbench {

// Klasa reprezentujaca robota IRp-6 na postumencie.

/*!
 * \brief class of EDP SwarmItFix head effector
 *
 * This head is built on top of the SPKM manipulator
 */
class effector : public common::motor_driven_effector
{
protected:

	const static int FESTO_ADRESS = 1;

	const static int NUMBER_OF_FESTO_GROUPS = 7;

	const static int TOTAL_NUMBER_OF_PINS_ACTIVATED_LIMIT = 3;

	//! Access to the CAN gateway unit
	boost::shared_ptr <canopen::gateway> gateway;

	//! festo shared ptr
	boost::shared_ptr <festo::cpv> cpv10;

	// Metoda tworzy modele kinematyczne dla robota IRp-6 na postumencie.
	/*!
	 * \brief method,  creates a list of available kinematic models for sbench effector.
	 *
	 * It will be used if any motor will be commanded to move. Then motor to joint transform will be implemented in kinematics.
	 */
	virtual void create_kinematic_models_for_given_robot(void);

	/*!
	 * \brief current pins state
	 *
	 * it is copied from desired in test_mode or read in hardware_mode
	 */
	lib::sbench::rbuffer current_pins_buf;

public:

	/*!
	 * \brief class constructor
	 *
	 * The attributes are initialized here.
	 */
	effector(common::shell &_shell);

	/*!
	 * \brief method to init voltage hardware
	 */
	void voltage_init();

	/*!
	 * \brief method to init preasure hardware
	 */
	void preasure_init();

	/*!
	 * \brief method to create threads other then EDP master thread.
	 *
	 * Here there is only one extra thread - reader_thread.
	 */
	void create_threads();

	void get_controller_state(lib::c_buffer &instruction);

	/*!
	 * \brief method to set position of the motors or joints
	 *
	 * It will be used if there will be any motor used.
	 */
	void move_arm(const lib::c_buffer &instruction); // przemieszczenie ramienia

	/*!
	 * \brief method to command voltage of pins
	 */
	void voltage_command(lib::sbench::c_buffer &instruction); // przemieszczenie ramienia

	/*!
	 * \brief method to command preasure in pins
	 */
	void preasure_command(lib::sbench::c_buffer &instruction); // przemieszczenie ramienia

	/*!
	 * \brief method to get position of the motors or joints
	 *
	 * It will be used if there will be any motor used.
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

	/*!
	 * \brief method to command reply of pins
	 */
	void voltage_reply();

	/*!
	 * \brief method to reply preasure in pins
	 */
	void preasure_reply();

	/*!
	 * \brief method to choose master_order variant
	 *
	 * IHere the single thread variant is chosen
	 */
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

	/*!
	 * \brief method to receive instruction from ecp of particular type
	 */
	lib::INSTRUCTION_TYPE variant_receive_instruction();

	/*!
	 * \brief method to reply to ecp with class of particular type
	 */
	void variant_reply_to_instruction();

	/*!
	 * \brief The particular type of instruction send form ECP to EDP
	 */
	lib::sbench::c_buffer instruction;

	/*!
	 * \brief The particular type of reply send form EDP to ECP
	 */
	lib::sbench::r_buffer reply;

private:
	const std::string dev_name;
	comedi_t *voltage_device; // device descriptor

	/*!
	 * \brief current and desired output data of festo controller
	 */
	std::bitset <8> current_output[NUMBER_OF_FESTO_GROUPS + 1], desired_output[NUMBER_OF_FESTO_GROUPS + 1];

};

} // namespace smb
} // namespace edp
} // namespace mrrocpp

#endif
