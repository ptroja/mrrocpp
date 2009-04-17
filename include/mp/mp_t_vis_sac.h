// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie vision force
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_VIS_SAC_H)
#define __MP_TASK_VIS_SAC_H

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace task {

class vis_sac: public task  {
protected:


public:
	
	vis_sac(lib::configurator &_config);

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);
  
};


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
