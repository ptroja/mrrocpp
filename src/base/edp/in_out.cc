#include <cstring>
#include <stdint.h>
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#else
#include <pthread.h>
#endif

#include "base/edp/in_out.h"

namespace mrrocpp {
namespace edp {
namespace common {

in_out_buffer::in_out_buffer()
{
    binary_input = 0;
    binary_output = 0;
    for (int i=0; i<8; i++)
    {
        analog_input[i]=0;
    }

    // inicjacja spinlockow
#ifdef __QNXNTO__
    memset( &input_spinlock, 0, sizeof(input_spinlock));
    memset( &output_spinlock, 0, sizeof(output_spinlock));
#else
    pthread_spin_init(&input_spinlock, PTHREAD_PROCESS_PRIVATE);
    pthread_spin_init(&output_spinlock, PTHREAD_PROCESS_PRIVATE);
#endif
    set_output_flag=false;
}

in_out_buffer::~in_out_buffer() {
#ifndef __QNXNTO__
	pthread_spin_destroy(&input_spinlock);
	pthread_spin_destroy(&output_spinlock);
#endif
}


// ustawienie wyjsc
void in_out_buffer::set_output(uint16_t out_value)
{
#ifdef __QNXNTO__
    InterruptLock
#else
    pthread_spin_lock
#endif
		(&output_spinlock);

    set_output_flag=true;   // aby f. obslugi przerwania wiedziala ze ma ustawic wyjscie
    binary_output=out_value;

#ifdef __QNXNTO__
    InterruptUnlock
#else
    pthread_spin_unlock
#endif
		(&output_spinlock);
}

// odczytanie wyjsc
void in_out_buffer::get_output(uint16_t *out_value)
{
#ifdef __QNXNTO__
    InterruptLock
#else
    pthread_spin_lock
#endif
		(&output_spinlock);

    *out_value=binary_output;

#ifdef __QNXNTO__
    InterruptUnlock
#else
    pthread_spin_unlock
#endif
		(&output_spinlock);
}


// ustawienie wejsc
void in_out_buffer::set_input (uint16_t binary_in_value, const uint8_t analog_in_table[])
{
#ifdef __QNXNTO__
    InterruptLock
#else
    pthread_spin_lock
#endif
		(&input_spinlock);

    binary_input=binary_in_value;		// wejscie binarne
    for (int i=0; i<8; i++)
    {
        analog_input[i]=analog_in_table[i];
    }

#ifdef __QNXNTO__
    InterruptUnlock
#else
    pthread_spin_unlock
#endif
		(&input_spinlock);
}


// odczytanie wejsc
void in_out_buffer::get_input (uint16_t *binary_in_value, uint8_t analog_in_table[])
{
#ifdef __QNXNTO__
    InterruptLock
#else
    pthread_spin_lock
#endif
		(&input_spinlock);

    *binary_in_value=binary_input;		// wejscie binarne
    for (int i=0; i<8; i++)
    {
        analog_in_table[i]=analog_input[i];
    }

#ifdef __QNXNTO__
    InterruptUnlock
#else
    pthread_spin_unlock
#endif
		(&input_spinlock);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
