// -------------------------------------------------------------------------
//                           mp_task_rcs_test.h
// Tester RCS - rozwiazywania kostki Rubika przy pomocy dwoch VSP
// autor: Jadwiga Salacka
// data: 04.04.2007
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_RCS_TEST_H)
#define __MP_TASK_RCS_TEST_H

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace task {

class mp_task_rcs_test : public mp_task  
{
public:

    // konstruktor
    mp_task_rcs_test(configurator &_config);
	
    // destruktor
    ~mp_task_rcs_test();

	// metody z mp
	void task_initialization(void);
	void main_task_algorithm(void);

}; // end : class mp_task_rcs_test


} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
