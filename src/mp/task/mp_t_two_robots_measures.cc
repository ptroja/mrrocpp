// ------------------------------------------------------------------------
// Proces:		MASTER PROCESS (MP)
// Plik:			task/mp_t_two_robots_measures.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Zadanie odpowiedzalne za zbieranie pomiarow
//				do weryfikacji kalibracji
//				- definicja metod klasy
//
// Autor:		tkornuta
// Data:		23.02.2007
// ------------------------------------------------------------------------

#include "mp/task/mp_t_two_robots_measures.h"
#include "mp/generator/mp_g_two_robots_measures.h"

namespace mrrocpp {
namespace mp {
namespace task {

// Zwrocenie obiektu - zadania.
task* return_created_mp_task(lib::configurator &_config)
{
	return new two_robots_measures(_config);
}

two_robots_measures::two_robots_measures(lib::configurator &_config) :
	task(_config)
{
	// Stworzenie generatora.
	rmg = new generator::two_robots_measures(*this);
	rmg->robot_m = robot_m;

	sr_ecp_msg->message("Two robots measurements task is ready for work.");
}

// Wlasciwe zadanie.
void two_robots_measures::main_task_algorithm(void)
{
	rmg->Move();
	sr_ecp_msg->message("Po move");
}


} // namespace task
} // namespace mp
} // namespace mrrocpp
