// ------------------------------------------------------------------------
// transformation thread by Y
// ------------------------------------------------------------------------

/********************************* INCLUDES *********************************/
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/mis_fun.h"
#include "base/edp/edp_e_motor_driven.h"
#include "base/edp/manip_trans_t.h"

/********************************* GLOBALS **********************************/

namespace mrrocpp {
namespace edp {
namespace common {

manip_trans_t::manip_trans_t(motor_driven_effector& _master) :
		master(_master)
{
	thread_id = boost::thread(boost::bind(&manip_trans_t::operator(), this));
}

void manip_trans_t::operator()()
{
	lib::set_thread_priority(pthread_self(), lib::PTHREAD_MAX_PRIORITY);

	while (1) {

		// oczekiwanie na zezwolenie ruchu od edp_master
		master_to_trans_synchroniser.wait();

		//domyslnie brak bledu boost
		error = boost::exception_ptr();

		// przekopiowanie instrukcji z bufora watku komunikacji z ECP (edp_master)

		current_cmd = tmp_cmd;
		//        master.current_instruction = master.instruction;

		try {
			// TODO: this thread is for handling special case of move_arm instruction;
			// all the othrer call can (and should...) be done from the main communication thread;
			// they do not need to be processed asynchronously.
			switch (current_cmd.trans_t_task)
			{
				case MT_GET_CONTROLLER_STATE:
					master.get_controller_state(current_cmd.instruction);
					trans_t_to_master_synchroniser.command();
					break;
				case MT_SET_ROBOT_MODEL:
					master.set_robot_model(current_cmd.instruction);
					trans_t_to_master_synchroniser.command();
					break;
				case MT_GET_ARM_POSITION:
					master.get_arm_position(current_cmd.trans_t_tryb, current_cmd.instruction);
					trans_t_to_master_synchroniser.command();
					break;
				case MT_GET_ALGORITHMS:
					master.get_algorithms();
					trans_t_to_master_synchroniser.command();
					break;
				case MT_SYNCHRONISE:
					master.synchronise();
					trans_t_to_master_synchroniser.command();
					break;
				case MT_MOVE_ARM:
					master.move_arm(current_cmd.instruction); // wariant dla watku edp_trans_t
					break;
				default: // blad: z reply_type wynika, e odpowied nie ma zawiera narzedzia
					break;
			}

		}

		// sekcja przechwytujaca bledy i przygotowujaca do ich rzucania w watku master

		catch (...) {
			error = boost::current_exception();
			printf("transformation thread unidentified_error\n");

			trans_t_to_master_synchroniser.command();
			// Wylapywanie niezdefiniowanych bledow
			// printf("zlapane cos");// by Y&W
		}

	}
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

