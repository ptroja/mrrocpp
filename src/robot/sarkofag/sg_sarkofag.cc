/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - styczen 2005
/* --------------------------------------------------------------------- */

#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "robot/sarkofag/sg_sarkofag.h"

// Klasa edp_sarkofag_effector.
#include "robot/sarkofag/edp_e_sarkofag.h"
#include "base/edp/reader.h"
// Klasa hardware_interface.
#include "robot/hi_moxa/hi_moxa.h"
// Klasa servo_buffer.

#include "robot/sarkofag/regulator_sarkofag.h"

namespace mrrocpp {
namespace edp {
namespace sarkofag {

/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer(effector &_master) :
	common::servo_buffer(_master), master(_master)
{

	synchro_axis_order[0] = 0;

	axe_inc_per_revolution[0] = INC_PER_REVOLUTION;
	synchro_step_coarse[0] = SYNCHRO_STEP_COARSE;
	synchro_step_fine[0] = SYNCHRO_STEP_FINE;

	thread_id = new boost::thread(boost::bind(&servo_buffer::operator(), this));
}
/*-----------------------------------------------------------------------*/

void servo_buffer::load_hardware_interface(void)
{

	// tablica pradow maksymalnych dla poszczegolnych osi
	//	int max_current[NUM_OF_SERVOS] = { SARKOFAG_AXIS_7_MAX_CURRENT };

	const std::vector <std::string>
			ports_vector(mrrocpp::lib::sarkofag::ports_strings, mrrocpp::lib::sarkofag::ports_strings
					+ mrrocpp::lib::sarkofag::LAST_MOXA_PORT_NUM + 1);
	hi
			= new hi_moxa::HI_moxa(master, mrrocpp::lib::sarkofag::LAST_MOXA_PORT_NUM, ports_vector, mrrocpp::lib::sarkofag::MAX_INCREMENT);
	hi->init();
	hi->set_parameter(0, hi_moxa::PARAM_MAXCURRENT, mrrocpp::lib::sarkofag::MAX_CURRENT_0);
	// utworzenie tablicy regulatorow
	// Serwomechanizm 1

	// regulator_ptr[1] = new NL_regulator_2 (0, 0, 0.71, 13./4, 12.57/4, 0.35);
	// kolumna dla sarkofag
	regulator_ptr[0] = new NL_regulator_8_sarkofag(0, 0, 0.39, 8.62 / 2., 7.89 / 2., 0.35, master);

	common::servo_buffer::load_hardware_interface();

}

/*-----------------------------------------------------------------------*/
void servo_buffer::get_all_positions(void)
{
	common::servo_buffer::get_all_positions();

	// przepisanie stanu regulatora chwytaka do bufora odpowiedzi dla EDP_master
	servo_data.gripper_reg_state = regulator_ptr[0]->get_reg_state();

}
/*-----------------------------------------------------------------------*/

} // namespace sarkofag
} // namespace edp
} // namespace mrrocpp
