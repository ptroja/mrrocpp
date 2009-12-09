// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __EDP_H
#define __EDP_H

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "kinematics/common/transformer_error.h"

namespace mrrocpp {
namespace edp {
namespace common {

class reader_buffer;
class master_trans_t_buffer;
class in_out_buffer;

class irp6s_postument_track_effector;

enum STATE { GET_STATE, GET_SYNCHRO, SYNCHRO_TERMINATED, GET_INSTRUCTION, EXECUTE_INSTRUCTION, WAIT, WAIT_Q };

enum TRANSLATION_ENUM { WITH_TRANSLATION, WITHOUT_TRANSLATION };

class System_error
{
    // Klasa bledow systemowych zawiazanych z komunikacja miedzyprocesowa
};

/**************************** master_trans_t_buffer *****************************/

enum MT_ORDER { MT_GET_CONTROLLER_STATE, MT_SET_RMODEL, MT_GET_ARM_POSITION, MT_GET_ALGORITHMS, MT_MOVE_ARM, MT_SYNCHRONISE};
enum ERROR_TYPE { NO_ERROR, Fatal_erroR, NonFatal_erroR_1, NonFatal_erroR_2, NonFatal_erroR_3, NonFatal_erroR_4, System_erroR};

class master_trans_t_buffer : public kinematic::common::transformer_error
{
private:
    sem_t master_to_trans_t_sem; // semafor pomiedzy edp_master a edp_trans
    sem_t trans_t_to_master_sem; // semafor pomiedzy edp_master a edp_trans

public:

    MT_ORDER trans_t_task;
    int trans_t_tryb;
    ERROR_TYPE error;

    // wskaznik na bledy (rzutowany na odpowiedni blad)
    void* error_pointer;

    master_trans_t_buffer();
    ~master_trans_t_buffer();

    int	master_to_trans_t_order(MT_ORDER nm_task, int nm_tryb);
    int	master_wait_for_trans_t_order_status();
    int	trans_t_to_master_order_status_ready();
    int	trans_t_wait_for_master_order();
};
/**************************** master_trans_t_buffer *****************************/


/**************************** IN_OUT_BUFFER *****************************/
class in_out_buffer
{
private:
    uint16_t binary_input;		// wejscie binarne
    uint8_t analog_input[8];		// wejscie analogowe - dla 8 kanalow

    uint16_t binary_output;		// wyjscie binarne
#ifdef __QNXNTO__
    intrspin_t
#else
    pthread_spinlock_t
#endif /* __QNXNTO__ */
		output_spinlock, input_spinlock; // spinlocki do wejścia/wyjscia

public:

    // konstruktor
    in_out_buffer();
    virtual ~in_out_buffer();

    bool set_output_flag; // flaga czy ustawic wyjcie na robota

    void set_output (const uint16_t *out_value);
    void get_output (uint16_t *out_value);
    void set_input (const uint16_t *binary_in_value, const uint8_t *analog_in_table);
    void get_input (uint16_t *binary_in_value, uint8_t *analog_in_table);
};
/**************************** IN_OUT_BUFFER *****************************/


} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
