// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie vision force
//
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_VIS_NN_H)
#define __MP_TASK_VIS_NN_H

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace task {

class vis_nn: public task  {

public:

	vis_nn(lib::configurator &_config);

	// methods for mp template
	void main_task_algorithm(void);

};


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
