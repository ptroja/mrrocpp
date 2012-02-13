/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <unistd.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"

// Klasa edp_irp6ot_effector.
#include "robot/irp6ot_m/edp_irp6ot_m_effector.h"
#include "base/edp/reader.h"
// Klasa hardware_interface.
//#include "robot/irp6p_m/hi_irp6p_m.h"
#include "robot/hi_moxa/hi_moxa.h"
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

	thread_id = boost::thread(boost::bind(&servo_buffer::operator(), this));
}
/*-----------------------------------------------------------------------*/

void servo_buffer::load_hardware_interface(void)
{
	const std::vector <std::string> ports_vector(mrrocpp::lib::irp6ot_m::ports_strings, mrrocpp::lib::irp6ot_m::ports_strings
			+ mrrocpp::lib::irp6ot_m::LAST_MOXA_PORT_NUM + 1);
	hi =
			new hi_moxa::HI_moxa(master, mrrocpp::lib::irp6ot_m::LAST_MOXA_PORT_NUM, ports_vector, mrrocpp::lib::irp6ot_m::MAX_INCREMENT);
	hi->init();

	hi->set_parameter(0, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::irp6ot_m::MAX_CURRENT_0);
	hi->set_parameter(1, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::irp6ot_m::MAX_CURRENT_1);
	hi->set_parameter(2, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::irp6ot_m::MAX_CURRENT_2);
	hi->set_parameter(3, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::irp6ot_m::MAX_CURRENT_3);
	hi->set_parameter(4, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::irp6ot_m::MAX_CURRENT_4);
	hi->set_parameter(5, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::irp6ot_m::MAX_CURRENT_5);
	hi->set_parameter(6, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::irp6ot_m::MAX_CURRENT_6);

	// utworzenie tablicy regulatorow
	// Serwomechanizm 1
	// regulator_ptr[0] = new NL_regulator_1 (0, 0, 0.64, 16.61/5., 15.89/5., 0.35);
	regulator_ptr[0] = new NL_regulator_1_irp6ot(0, 0, 0, 0.333, 6.2, 5.933, 0.35, master);
	// Serwomechanizm 2
	// regulator_ptr[1] = new NL_regulator_2 (0, 0, 0.71, 13./4, 12.57/4, 0.35);
	regulator_ptr[1] = new NL_regulator_2_irp6ot(1, 0, 0, 0.429, 6.834, 6.606, 0.35, master);
	// Serwomechanizm 3
	regulator_ptr[2] = new NL_regulator_3_irp6ot(2, 0, 0, 0.64, 9.96 / 4, 9.54 / 4, 0.35, master);
	// Serwomechanizm 4
	// regulator_ptr[3] = new NL_regulator_4 (0, 0, 0.62, 9.85/4, 9.39/4, 0.35);
	regulator_ptr[3] = new NL_regulator_4_irp6ot(3, 0, 0, 0.333, 5.693, 5.427, 0.35, master);
	// Serwomechanizm 5
	regulator_ptr[4] = new NL_regulator_5_irp6ot(4, 0, 0, 0.56, 7.98 / 2, 7.55 / 2, 0.35, master);
	// Serwomechanizm 6
	// regulator_ptr[5] = new NL_regulator_6 (0, 0, 0.3079*2, 0.6, 0.6, 0.35);
	regulator_ptr[5] = new NL_regulator_6_irp6ot(5, 0, 0, 0.39, 8.62 / 2., 7.89 / 2., 0.35, master);

	regulator_ptr[6] = new NL_regulator_7_irp6ot(6, 0, 0, 0.39, 8.62 / 2., 7.89 / 2., 0.35, master);

	common::servo_buffer::load_hardware_interface();
}

} // namespace irp6ot
} // namespace edp
} // namespace mrrocpp
