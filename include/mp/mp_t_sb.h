// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie vision force
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_SB_H)
#define __MP_TASK_SB_H

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace task {

class sb: public base  {
protected:


public:
	
	sb(configurator &_config);

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);
  
};


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
