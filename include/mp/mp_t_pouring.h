// -------------------------------------------------------------------------
//                            mp_t_pouring.h
// Definicje struktur danych i metod dla procesow MP - zadanie przelewania
//  wersja z generatorami uruchaminami na poziomie ECP
// Autor: Przemek Pilacinski
// Ostatnia modyfikacja: styczen 2008
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_POURING)
#define __MP_TASK_POURING

#include "mp/mp.h"
#include "ecp_mp/ecp_mp_t_pouring.h"

class mp_task_pouring : public mp_task  
{
protected:
	bool break_state;

public:
    // konstruktor
    mp_task_pouring(configurator &_config);
	
    ~mp_task_pouring();

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);

	bool approach(void);
	bool grab(void);
	bool weight(void);
	bool meet(void);
	bool pour(void);
	bool go_back(void);
	bool put_back(void);
	bool depart(void);

}; // end : class mp_task_pouring
#endif
