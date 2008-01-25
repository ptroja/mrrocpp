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

class mp_two_robots_measures_task : public mp_task  
{
protected:
	mp_two_robots_measures_generator *rmg;
public:
	// Object initialization.
	void task_initialization(void);
	// Exact task algorithm.
	void main_task_algorithm(void);
};//: mp_task_two_robots_measures

#endif
