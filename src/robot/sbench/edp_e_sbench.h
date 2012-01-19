/*!
 * @file
 * @brief File containing the declaration of edp::sbench::effector class.
 *
 * @author yoyek
 *
 * @ingroup sbench
 */

#ifndef __EDP_E_SBENCH_H
#define __EDP_E_SBENCH_H

#include <string>
#include <bitset>

#include "base/edp/edp_e_motor_driven.h"
#include "dp_sbench.h"

#include "../canopen/gateway.h"
#include "../festo/cpv.h"

namespace mrrocpp {
namespace edp {
namespace sbench {

/*!
 * \brief class of EDP SwarmItFix sbench effector
 *
 * This sbench is built on top of the SPKM manipulator
 *
 * See http://www.festo.com/net/SupportPortal/Downloads/51705/526410g1.pdf
 * for details about CAN communication.
 */
class effector : public common::motor_driven_effector
{
protected:

	//! address of the festo valve block
	const static int FESTO_ADRESS = 1;

	//! the number of logical adress blocks in the festo valve block
	const static int NUMBER_OF_FESTO_GROUPS = 7;

	//! the maximal number of pins that can be cleaned at once (due to the festo block properties)
	const static int CLEANING_PINS_ACTIVATED_LIMIT = 4;

	//! the maximal number of pins that can be activated (supplied with electricity) at once (due to the relays properties)
	const static int VOLTAGE_PINS_ACTIVATED_LIMIT = 6;

	//! the relays hardware activation flag set by the same labeled configuration field
	bool power_supply;

	//! the festo hardware active flag set by the same labeled configuration field
	bool cleaning;

	//! Access to the CAN gateway unit
	boost::shared_ptr <canopen::gateway> gateway;

	//! festo shared ptr
	boost::shared_ptr <festo::cpv> cpv10;

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
	 * \brief Disconnects the power supply device.
	 */
	~effector();

	/*!
	 * Empty.
	 */
	virtual void create_kinematic_models_for_given_robot(void);

	/*!
	 * \brief method to init voltage hardware
	 */
	void power_supply_init();

	/*!
	 * \brief method to init preasure hardware
	 */
	void cleaning_init();

	/*!
	 * \brief method to create threads other then EDP master thread.
	 *
	 * Here there is only one extra thread - reader_thread.
	 */
	void create_threads();

	/*!
	 * \brief The method checks the initial state of the controller.
	 * Bench is synchronised at the start.
	 */
	void get_controller_state(lib::c_buffer &instruction_);

	/*!
	 * \brief method to set position of the motors or joints
	 *
	 */
	void move_arm(const lib::c_buffer &instruction_);

	/*!
	 * \brief method to command voltage of pins
	 */
	void power_supply_command();

	/*!
	 * \brief method to command preasure in pins
	 */
	void cleaning_command();

	/*!
	 * \brief method to get position of the motors or joints
	 *
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction_);

	/*!
	 * \brief method to reply of pins
	 */
	void power_supply_reply();

	/*!
	 * \brief method to reply preasure in pins
	 */
	void cleaning_reply();

	/*!
	 * \brief method to choose master_order variant
	 *
	 * Here the single thread variant is chosen.
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
	 * @brief checks if the configuration file let the festo hardware to run
	 * or it is in test mode
	 * @return hardware_active
	 */
	bool cleaning_active();

	/*!
	 * @brief checks if the configuration file let the relays hardware to run
	 * or it is in test mode
	 * @return hardware_active
	 */
	bool power_supply_active();


	/*!
	 * \brief The particular type of instruction send form ECP to EDP
	 */
	lib::sbench::c_buffer instruction;

	/*!
	 * \brief The particular type of reply send form EDP to ECP
	 */
	lib::sbench::r_buffer reply;

private:

	/*!
	 * \brief device name for advantech card
	 */
	const std::string dev_name;

	/*!
	 * \brief  device descriptor for advantech card
	 */
	comedi_t *power_supply_device;

	/*!
	 * \brief current and desired output data of festo controller
	 */
	std::bitset <8> current_output[NUMBER_OF_FESTO_GROUPS + 1], desired_output[NUMBER_OF_FESTO_GROUPS + 1];

};

} // namespace sbench
} // namespace edp
} // namespace mrrocpp

#endif
