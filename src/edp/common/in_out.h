// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __IN_OUT_BUFFER_H
#define __IN_OUT_BUFFER_H

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
