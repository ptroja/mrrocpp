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


public:
    // konstruktor
    mp_task_pouring(configurator &_config);
	
    ~mp_task_pouring();

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);

	void approach(void);
	void grab(void);
	void weight(void);
	void meet(void);
	void pour(void);
	void go_back(void);
	void put_back(void);
	void depart(void);

}; // end : class mp_task_pouring
#endif
