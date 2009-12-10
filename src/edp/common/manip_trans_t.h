
// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __MANIP_TRANS_T_H
#define __MANIP_TRANS_T_H

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

#include "edp/common/trans_t.h"

namespace mrrocpp {
namespace edp {
namespace common {


/**************************** trans_t *****************************/

class manip_and_conv_effector;

class manip_trans_t : public trans_t
{
private:
    manip_and_conv_effector &master;

public:
    static void *trans_thread_start(void* arg);
    void *trans_thread(void* arg);

    manip_trans_t(manip_and_conv_effector& _master);
    ~manip_trans_t();

};
/**************************** trans_t *****************************/




} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
