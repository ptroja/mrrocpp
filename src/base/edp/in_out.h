// -------------------------------------------------------------------------
//                                   edp.h
// Definicje struktur danych i metod dla procesu EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __IN_OUT_BUFFER_H
#define __IN_OUT_BUFFER_H

#include <stdint.h>
#if (defined(__APPLE__) && defined(__MACH__))
#include <libkern/OSAtomic.h>
#define SPIN_LOCK	OSSpinLockLock
#define SPIN_UNLOCK	OSSpinLockUnlock
#else
#include <pthread.h>
#define SPIN_LOCK	pthread_spin_lock
#define SPIN_UNLOCK	pthread_spin_unlock
#endif

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
#if (defined(__APPLE__) && defined(__MACH__))
    OSSpinLock
#else
    pthread_spinlock_t
#endif
		output_spinlock, input_spinlock; // spinlocki do wejścia/wyjscia

public:

    // konstruktor
    in_out_buffer();
    virtual ~in_out_buffer();

    bool set_output_flag; // flaga czy ustawic wyjcie na robota

    void set_output (uint16_t out_value);
    void get_output (uint16_t *out_value);

    void set_input (uint16_t binary_in_value, const uint8_t analog_in_table[]);
    void get_input (uint16_t *binary_in_value, uint8_t analog_in_table[]);
};
/**************************** IN_OUT_BUFFER *****************************/


} // namespace common
} // namespace edp
} // namespace mrrocpp

#endif
