#ifndef MP_GENERATOR_H_
#define MP_GENERATOR_H_


#include "mp/mp_robot.h"
#include "ecp_mp/ecp_mp_generator.h"





#include <map>

// class common::mp_task;
// class mp_robot;

namespace mrrocpp {
namespace mp {
namespace generator {



class generator : public ecp_mp::generator::generator
{
	protected:

		int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)

	public:

		task::task& mp_t;

		// Funkcja ruchu
		void Move (void);

		// zbior obejmujacy mozliwe stany obiektu klasy generator_set
		enum GEN_SET_PHASE {
		    BEFORE_FIRST_STEP,
		    AFTER_STEP,
		    AFTER_INITIATE_READING,
		    WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION,
		    AFTER_EXECUTE_MOTION,
		    AFTER_GET_READING,
		    GS_FINISHED
		};

		//! faza w ktorej znajduje sie generator
		GEN_SET_PHASE phase;

		//! czy nowy puls zostal sprawdzony (wykorzystywane w scheduler_run() )
		bool new_pulse_checked;

		/*!
		 * okresla czy przed next step Move ma sie zawieszac w oczekwianiu na puls z ECP;
		 * wykorzystywane przy luznej i sporadycznej wspolpracy robotow.
		 */
		bool wait_for_ECP_pulse;

		//! mapa wszystkich robotow
		std::map <lib::ROBOT_ENUM, common::robot*> robot_m;

		generator(task::task& _mp_task);

		void re_run(void); // powrot do stanu wyjsciowego

		//! Kopiuje dane z robotow do generatora
		void copy_data(std::map <lib::ROBOT_ENUM, common::robot*>& _robot_m);

		//! Kopiuje polecenie stworzone w generatorze do robotow
		void copy_generator_command (std::map <lib::ROBOT_ENUM, common::robot*>& _robot_m);

		//! Klasa obslugi bledow generatora na poziomie MP
		class MP_error
		{
			public:
				const lib::ERROR_CLASS error_class;
				const uint64_t mp_error;

				MP_error(lib::ERROR_CLASS err0, uint64_t err1) :
					error_class(err0), mp_error(err1)
				{
				}
		};
};
} // namespace common
} // namespace mp
} // namespace mrrocpp
#endif /*MP_GENERATOR_H_*/
