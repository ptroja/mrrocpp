/* --------------------------------------------------------------------- */
/*                          SERVO_GROUP Process                          */
// ostatnia modyfikacja - 13.01.2011
/* --------------------------------------------------------------------- */

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

// Klasa edp_polycrank_effector.
//#include "robot/conveyor/edp_conveyor_effector.h"
#include "robot/polycrank/edp_e_polycrank.h"
// Klasa hardware_interface.
#include "robot/polycrank/hi_polycrank.h"
// Klasa servo_buffer.
#include "robot/polycrank/sg_polycrank.h"
#include "robot/polycrank/regulator_polycrank.h"

namespace mrrocpp {
namespace edp {
namespace polycrank {

/*-----------------------------------------------------------------------*/
servo_buffer::servo_buffer(effector &_master) :
	common::servo_buffer(_master), master(_master)
{

	for (int j = 0; j < lib::polycrank::NUM_OF_SERVOS; j++) {
		axe_inc_per_revolution[j] = INC_PER_REVOLUTION;
	}

	//printf("servo_buffer\n");
	thread_id = new boost::thread(boost::bind(&servo_buffer::operator(), this));
}
/*
servo_buffer::servo_buffer(effector &_master) :
	common::servo_buffer(_master), master(_master)
{
	synchro_axis_order[0] = 0;
	axe_inc_per_revolution[0] = AXIS_7_INC_PER_REVOLUTION;
	synchro_step_coarse[0] = AXIS_7_SYNCHRO_STEP_COARSE;
	synchro_step_fine[0] = AXIS_7_SYNCHRO_STEP_FINE;

	thread_id = new boost::thread(boost::bind(&servo_buffer::operator(), this));
}*/

void servo_buffer::load_hardware_interface(void)
{
	// tablica pradow maksymalnych dla poszczegolnych osi
	//int max_current[lib::conveyor::NUM_OF_SERVOS] = { AXIS_1_MAX_CURRENT };

	const std::vector<std::string> ports_vector(mrrocpp::lib::polycrank::ports_strings,
				mrrocpp::lib::polycrank::ports_strings+mrrocpp::lib::polycrank::LAST_MOXA_PORT_NUM+1);
	hi = new hi_moxa::HI_moxa(master, mrrocpp::lib::polycrank::LAST_MOXA_PORT_NUM, ports_vector, NULL);
	hi->init();

	// conveyor uruchamia sie jako zsynchronizowany - ustawic parametr na karcie sterownika
	hi->set_parameter(0, hi_moxa::PARAM_SYNCHRONIZED, 1);

	// utworzenie tablicy regulatorow

	//regulator_ptr[0] = new NL_regulator_1_polycrank(0, 0, 0.333, 6.2, 5.933, 0.35, master); // tasmociag dla irp6 postument
	//regulator_ptr[1] = new NL_regulator_2_polycrank(0, 0, 0.64, 9.96 / 4, 9.54 / 4, 0.35, master);

	regulator_ptr[0] = new NL_regulator_polycrank(0, 0, 0, 0, 0, 0, master);
	regulator_ptr[1] = new NL_regulator_polycrank(0, 0, 0, 0, 0, 0, master);
	regulator_ptr[2] = new NL_regulator_polycrank(0, 0, 0, 0, 0, 0, master);
	regulator_ptr[3] = new NL_regulator_polycrank(0, 0, 0, 0, 0, 0, master);
	regulator_ptr[4] = new NL_regulator_polycrank(0, 0, 0, 0, 0, 0, master);
	regulator_ptr[5] = new NL_regulator_polycrank(0, 0, 0, 0, 0, 0, master);
	regulator_ptr[6] = new NL_regulator_polycrank(0, 0, 0, 0, 0, 0, master);

	common::servo_buffer::load_hardware_interface();
}

/*
void servo_buffer::synchronise(void)
{
	//common::regulator* crp = NULL; // wskaznik aktualnie synchronizowanego napedu

	double synchro_step = 0.0; // zadany przyrost polozenia

	if (master.robot_test_mode) {
		// W.S. Tylko przy testowaniu
		clear_reply_status();
		clear_reply_status_tmp();
		reply_to_EDP_MASTER();
		return;
	}

	// zerowanie regulatorow
	for (int j = 0; j < lib::conveyor::NUM_OF_SERVOS; j++) {
		crp = regulator_ptr[j];
		crp->clear_regulator();
		hi->reset_position(j);
	}

	// zatrzymanie na chwile robota
	for (int j = 0; j < lib::conveyor::NUM_OF_SERVOS; j++) {
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
*/

} // namespace polycrank
} // namespace edp
} // namespace mrrocpp

