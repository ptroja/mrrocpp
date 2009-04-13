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

namespace mrrocpp {
namespace mp {
namespace task {

class pouring : public base  
{
protected:


public:
    // konstruktor
    pouring(lib::configurator &_config);
	
    ~pouring();

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

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
