// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie vision force
//
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_VIS_SAC_LX_H)
#define __MP_TASK_VIS_SAC_LX_H

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace task {

class vis_sac_lx: public task  {
public:

	vis_sac_lx(lib::configurator &_config);

	// methods for mp template
	void main_task_algorithm(void);

};


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
