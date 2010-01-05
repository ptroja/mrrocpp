
// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __TRANS_T_H
#define __TRANS_T_H

#include <stdint.h>
#include <semaphore.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "kinematics/common/transformer_error.h"
#include "edp/common/edp.h"
#include "edp/common/edp_extension_thread.h"

namespace mrrocpp {
namespace edp {
namespace common {

class effector;

/**************************** trans_t *****************************/

class trans_t : public kinematics::common::transformer_error, boost::noncopyable
{
private:
    sem_t master_to_trans_t_sem; // semafor pomiedzy edp_master a edp_trans
    sem_t trans_t_to_master_sem; // semafor pomiedzy edp_master a edp_trans
    effector &master;

protected:
    boost::thread *thread_id;

public:
    MT_ORDER trans_t_task;
    int trans_t_tryb;
    ERROR_TYPE error;

    virtual void operator()() = 0;

    // wskaznik na bledy (rzutowany na odpowiedni blad)
    void* error_pointer;

    trans_t(effector& _master);
    ~trans_t();

    void master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb);
    int	trans_t_to_master_order_status_ready();
    int	trans_t_wait_for_master_order();
};
/**************************** trans_t *****************************/




} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
