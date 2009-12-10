
// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __MASTER_TRANS_T_BUFFER_H
#define __MASTER_TRANS_T_BUFFER_H

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "kinematics/common/transformer_error.h"
#include "edp/common/edp.h"

namespace mrrocpp {
namespace edp {
namespace common {

class manip_and_conv_effector;

/**************************** master_trans_t_buffer *****************************/



class master_trans_t_buffer : public kinematic::common::transformer_error
{
private:
    sem_t master_to_trans_t_sem; // semafor pomiedzy edp_master a edp_trans
    sem_t trans_t_to_master_sem; // semafor pomiedzy edp_master a edp_trans
    manip_and_conv_effector &master;

public:
    static void *trans_thread_start(void* arg);
    void *trans_thread(void* arg);
	   pthread_t trans_t_tid;
    MT_ORDER trans_t_task;
    int trans_t_tryb;
    ERROR_TYPE error;

    // wskaznik na bledy (rzutowany na odpowiedni blad)
    void* error_pointer;

    master_trans_t_buffer(manip_and_conv_effector& _master);
    ~master_trans_t_buffer();

    int	master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb);
    int	master_wait_for_trans_t_order_status();
    int	trans_t_to_master_order_status_ready();
    int	trans_t_wait_for_master_order();
};
/**************************** master_trans_t_buffer *****************************/




} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
