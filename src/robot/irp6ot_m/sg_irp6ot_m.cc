/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <cstdio>
#include <cstdlib>
#include <unistd.h>

// Klasa edp_irp6ot_effector.
#include "robot/irp6ot_m/edp_irp6ot_m_effector.h"
#include "base/edp/reader.h"
// Klasa hardware_interface.
#include "robot/irp6ot_m/hi_irp6ot_m.h"
// Klasa servo_buffer.
#include "robot/irp6ot_m/sg_irp6ot_m.h"
#include "robot/irp6ot_m/regulator_irp6ot_m.h"

namespace mrrocpp {
namespace edp {
namespace irp6ot_m {

/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer(effector &_master) :
	common::servo_buffer(_master), master(_master)
{
	for (int j = 0; j < master.number_of_servos; j++) {
		synchro_axis_order[j] = ((j + SYN_INIT_AXE) % (master.number_of_servos));
		switch (j)
		{
			case GRIPPER_TURN_AXE:
				axe_inc_per_revolution[j] = AXIS_6_INC_PER_REVOLUTION;

				synchro_step_coarse[j] = AXIS_6_SYNCHRO_STEP_COARSE;
				synchro_step_fine[j] = AXIS_6_SYNCHRO_STEP_FINE;
				break;
			default:
				axe_inc_per_revolution[j] = AXIS_0_TO_5_INC_PER_REVOLUTION;
				synchro_step_coarse[j] = AXIS_0_TO_5_SYNCHRO_STEP_COARSE;
				synchro_step_fine[j] = AXIS_0_TO_5_SYNCHRO_STEP_FINE;

				break;
		}
	}

	thread_id = new boost::thread(boost::bind(&servo_buffer::operator(), this));
}
/*-----------------------------------------------------------------------*/

void servo_buffer::load_hardware_interface(void)
{
	// tablica pradow maksymalnych d;a poszczegolnych osi
	int
			max_current[lib::irp6ot_m::NUM_OF_SERVOS] =

					{ AXIS_1_MAX_CURRENT, AXIS_2_MAX_CURRENT, AXIS_3_MAX_CURRENT, AXIS_4_MAX_CURRENT, AXIS_5_MAX_CURRENT, AXIS_6_MAX_CURRENT, AXIS_7_MAX_CURRENT };

	hi
			= new hardware_interface(master, IRQ_REAL, INT_FREC_DIVIDER, HI_RYDZ_INTR_TIMEOUT_HIGH, FIRST_SERVO_PTR, INTERRUPT_GENERATOR_SERVO_PTR, ISA_CARD_OFFSET, max_current);
	hi->init();

	// utworzenie tablicy regulatorow
	// Serwomechanizm 1
	// regulator_ptr[0] = new NL_regulator_1 (0, 0, 0.64, 16.61/5., 15.89/5., 0.35);
	regulator_ptr[0] = new NL_regulator_1_irp6ot(0, 0, 0.333, 6.2, 5.933, 0.35, master);
	// Serwomechanizm 2
	// regulator_ptr[1] = new NL_regulator_2 (0, 0, 0.71, 13./4, 12.57/4, 0.35);
	regulator_ptr[1] = new NL_regulator_2_irp6ot(0, 0, 0.429, 6.834, 6.606, 0.35, master);
	// Serwomechanizm 3
	regulator_ptr[2] = new NL_regulator_3_irp6ot(0, 0, 0.64, 9.96 / 4, 9.54 / 4, 0.35, master);
	// Serwomechanizm 4
	// regulator_ptr[3] = new NL_regulator_4 (0, 0, 0.62, 9.85/4, 9.39/4, 0.35);
	regulator_ptr[3] = new NL_regulator_4_irp6ot(0, 0, 0.333, 5.693, 5.427, 0.35, master);
	// Serwomechanizm 5
	regulator_ptr[4] = new NL_regulator_5_irp6ot(0, 0, 0.56, 7.98 / 2, 7.55 / 2, 0.35, master);
	// Serwomechanizm 6
	// regulator_ptr[5] = new NL_regulator_6 (0, 0, 0.3079*2, 0.6, 0.6, 0.35);
	regulator_ptr[5] = new NL_regulator_6_irp6ot(0, 0, 0.39, 8.62 / 2., 7.89 / 2., 0.35, master);

	regulator_ptr[6] = new NL_regulator_7_irp6ot(0, 0, 0.39, 8.62 / 2., 7.89 / 2., 0.35, master);

	common::servo_buffer::load_hardware_interface();
}

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp
