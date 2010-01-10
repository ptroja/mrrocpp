/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Klasa edp_conveyor_effector.
#include "edp/conveyor/edp_conveyor_effector.h"
// Klasa hardware_interface.
#include "edp/conveyor/hi_conv.h"
// Klasa servo_buffer.
#include "edp/conveyor/sg_conv.h"
#include "edp/conveyor/regulator_conv.h"

namespace mrrocpp {
namespace edp {
namespace conveyor {

/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer(effector &_master) :
	common::servo_buffer(_master), master(_master)
{
	for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) {
		axe_inc_per_revolution[j] = IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION;
	}

	thread_id = new boost::thread(boost::bind(&servo_buffer::operator(), this));
}

/*-----------------------------------------------------------------------*/

void servo_buffer::load_hardware_interface(void)
{
	// tablica pradow maksymalnych dla poszczegolnych osi
	int max_current[CONVEYOR_NUM_OF_SERVOS] = { CONVEYOR_AXIS_1_MAX_CURRENT };

	hi
			= new hardware_interface(master, IRQ_REAL, INT_FREC_DIVIDER, HI_RYDZ_INTR_TIMEOUT_HIGH, FIRST_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR, ISA_CARD_OFFSET, max_current);

	hi->init();
	// utworzenie tablicy regulatorow

	// Serwomechanizm 1
	regulator_ptr[0] = new NL_regulator_1_conv(0, 0, 0.333, 6.2, 5.933, 0.35, master); // tasmociag dla irp6 postument

	send_after_last_step = false;
	clear_reply_status();
	clear_reply_status_tmp();

	for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) {
		command.parameters.move.abs_position[j] = 0.0;
	}
}

/*------------- * ----------------------------------------------------------*/

void servo_buffer::synchronise(void)
{
	common::regulator* crp = NULL; // wskaznik aktualnie synchronizowanego napedu

	double synchro_step = 0.0; // zadany przyrost polozenia

	if (master.test_mode) {
		// W.S. Tylko przy testowaniu
		clear_reply_status();
		clear_reply_status_tmp();
		reply_to_EDP_MASTER();
		return;
	}

	// zerowanie regulatorow
	for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) {
		crp = regulator_ptr[j];
		crp->clear_regulator();
		hi->reset_position(j);
	}

	// zatrzymanie na chwile robota
	for (int j = 0; j < CONVEYOR_NUM_OF_SERVOS; j++) {
		synchro_step = 0.0;
		crp = regulator_ptr[j];
		crp->insert_new_step(synchro_step);
	}

	for (int j = 0; j < 25; j++)
		Move_1_step();

	//	kk = 0;
	clear_reply_status();
	clear_reply_status_tmp();

	reply_to_EDP_MASTER();
	return;
}

/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
void servo_buffer::get_all_positions(void)
{
	// Przepisanie aktualnych polozen servo do pakietu wysylkowego
	for (int i = 0; i < CONVEYOR_NUM_OF_SERVOS; i++) {
		// przyrost polozenia w impulsach
		servo_data.abs_position[i] = hi->get_position(i) * (2*M_PI) / IRP6_POSTUMENT_AXIS_0_TO_5_INC_PER_REVOLUTION;
		servo_data.position[i] = regulator_ptr[i]->get_position_inc(1);
		servo_data.current[i] = regulator_ptr[i]->get_meassured_current();
		servo_data.PWM_value[i] = regulator_ptr[i]->get_PWM_value();
		servo_data.algorithm_no[i] = regulator_ptr[i]->get_algorithm_no();
		servo_data.algorithm_parameters_no[i] = regulator_ptr[i]->get_algorithm_parameters_no();
	}
}
/*-----------------------------------------------------------------------*/

} // namespace conveyor

namespace common {

servo_buffer* return_created_servo_buffer(manip_and_conv_effector &_master)
{
	return new conveyor::servo_buffer((conveyor::effector &) (_master));
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

