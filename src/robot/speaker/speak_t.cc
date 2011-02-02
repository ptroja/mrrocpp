// ------------------------------------------------------------------------
// transformation thread by Y
// ostatnia modyfikacja: styczen 2005
// ------------------------------------------------------------------------

#include <iostream>

#include "base/lib/com_buf.h"
#include "base/lib/mis_fun.h"

#include "robot/speaker/edp_speaker_effector.h"
#include "robot/speaker/speak_t.h"

#include "base/lib/exception.h"

using namespace mrrocpp::lib::exception;

namespace mrrocpp {
namespace edp {
namespace speaker {

speak_t::speak_t(effector& _master) :
	master(_master)
{
	thread_id = boost::thread(boost::bind(&speak_t::operator(), this));
}

void speak_t::operator()()
{
	lib::set_thread_priority(pthread_self(), lib::QNX_MAX_PRIORITY - 10);

	master.initialize_incorrect = (master.init() == -1);

	while (1) {
		master_to_trans_synchroniser.wait();

		try {
			switch (trans_t_task)
			{
				case common::MT_GET_ARM_POSITION:
					master.get_spoken(trans_t_tryb, instruction);
					trans_t_to_master_synchroniser.command();
					break;
				case common::MT_MOVE_ARM:
					trans_t_to_master_synchroniser.command();
					master.speak(instruction);
					break;
				default:
					break;
			}
		}

		// sekcja przechwytujaca bledy i przygotowujaca do ich rzucania w watku master
		catch (NonFatal_error_1 & nfe) {
			error_pointer = malloc(sizeof(nfe));
			memcpy(error_pointer, &nfe, sizeof(nfe));
			error_pointer = &nfe;
			error = common::NonFatal_erroR_1;
			trans_t_to_master_synchroniser.command();
		} // end: catch(NonFatal_error_1 nfe)

		catch (NonFatal_error_2 & nfe) {
			error_pointer = malloc(sizeof(nfe));
			memcpy(error_pointer, &nfe, sizeof(nfe));
			error_pointer = &nfe;
			error = common::NonFatal_erroR_2;
			trans_t_to_master_synchroniser.command();
		} // end: catch(NonFatal_error_2 nfe)

		catch (NonFatal_error_3 & nfe) {
			error_pointer = malloc(sizeof(nfe));
			memcpy(error_pointer, &nfe, sizeof(nfe));
			error_pointer = &nfe;
			error = common::NonFatal_erroR_3;
			trans_t_to_master_synchroniser.command();
		} // end: catch(NonFatal_error_3 nfe)

		catch (NonFatal_error_4 & nfe) {
			error_pointer = malloc(sizeof(nfe));
			memcpy(error_pointer, &nfe, sizeof(nfe));
			error = common::NonFatal_erroR_4;
			trans_t_to_master_synchroniser.command();
		} // end: catch(NonFatal_error nfe4)

		catch (Fatal_error & fe) {
			error_pointer = malloc(sizeof(fe));
			memcpy(error_pointer, &fe, sizeof(fe));
			error = common::Fatal_erroR;
			trans_t_to_master_synchroniser.command();
		} // end: catch(Fatal_error fe)

		catch (System_error & fe) {
			error_pointer = malloc(sizeof(fe));
			memcpy(error_pointer, &fe, sizeof(fe));
			error = common::System_erroR;
			trans_t_to_master_synchroniser.command();
		} // end: catch(System_error fe)

		catch (...) { // Dla zewnetrznej petli try
			std::cerr << "transformation thread: unidentified_error" << std::endl;

			trans_t_to_master_synchroniser.command();
			// Wylapywanie niezdefiniowanych bledow
		}

	} // end while
}

} // namespace common
} // namespace edp
} // namespace mrrocpp

