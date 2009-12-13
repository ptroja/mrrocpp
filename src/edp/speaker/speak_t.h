
// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __SPEAK_T_H
#define __SPEAK_T_H

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

#include "edp/common/trans_t.h"

namespace mrrocpp {
namespace edp {
namespace speaker {


/**************************** trans_t *****************************/

class effector;

class speak_t : public common::trans_t
{
private:
	effector &master;

public:
    static void *thread_start(void* arg);
    void *thread_main_loop(void* arg);

    speak_t(effector& _master);
    ~speak_t();

};
/**************************** trans_t *****************************/


} // namespace speaker
} // namespace edp
} // namespace mrrocpp

#endif
