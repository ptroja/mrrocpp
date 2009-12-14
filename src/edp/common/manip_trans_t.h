
// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __MANIP_TRANS_T_H
#define __MANIP_TRANS_T_H

#include "edp/common/trans_t.h"

namespace mrrocpp {
namespace edp {
namespace common {

class manip_and_conv_effector;

class manip_trans_t : public trans_t
{
private:
    manip_and_conv_effector &master;

public:
    static void *thread_start(void* arg);
    void *thread_main_loop(void* arg);
    void create_thread(void);
    manip_trans_t(manip_and_conv_effector& _master);
};

} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
