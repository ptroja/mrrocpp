// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie powielania rysunku
// 
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_PR_H)
#define __MP_TASK_PR_H

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace task {

class pr: public base  {
protected:
	
	void mp_short_move_up(void);
	
public:
	
	pr(configurator &_config);

	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);
  
};


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
