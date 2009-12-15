// ------------------------------------------------------------------------
// Proces:		MASTER PROCESS (MP)
// Plik:			task/mp_t_two_robots_measures.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		Zadanie odpowiedzalne za zbieranie pomiarow
//				do weryfikacji kalibracji
//				- deklaracja klasy
//
// Autor:		tkornuta
// Data:		23.02.2007
// ------------------------------------------------------------------------

#if !defined(__MP_TASK_TWO_ROBOTS_MEASURES)
#define __MP_TASK_TWO_ROBOTS_MEASURES

#include "mp/mp.h"
#include "mp/generator/mp_g_two_robots_measures.h"

namespace mrrocpp {
namespace mp {
namespace task {

class two_robots_measures : public task
{
protected:
	generator::two_robots_measures *rmg;
public:

	two_robots_measures(lib::configurator &_config);

	// Exact task algorithm.
	void main_task_algorithm(void);

};//: mp_task_two_robots_measures

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
