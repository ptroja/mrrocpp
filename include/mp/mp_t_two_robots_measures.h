// ------------------------------------------------------------------------
// Proces:		MASTER PROCESS (MP)
// Plik:			mp_t_two_robots_measures.h
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
#include "mp/mp_g_two_robots_measures.h"

namespace mrrocpp {
namespace mp {
namespace task {

class two_robots_measures : public base  
{
protected:
	generator::two_robots_measures *rmg;
public:
	
	two_robots_measures(configurator &_config);
	
	// Object initialization.
	void task_initialization(void);
	// Exact task algorithm.
	void main_task_algorithm(void);
};//: mp_task_two_robots_measures


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
