// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_irp6s_and_conv.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Metody wspolne dla robotow IRp-6 oraz tasmociagu
// 				- definicja metod klasy edp_irp6s_and_conv_effector
//
// Autor:		tkornuta
// Data:		14.01.2007
// -------------------------------------------------------------------------

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <signal.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <errno.h>
#include <pthread.h>
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#endif
#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "edp/common/in_out.h"

namespace mrrocpp {
namespace edp {
namespace common {


/**************************** IN_OUT_BUFFER *****************************/

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
void in_out_buffer::set_output(const uint16_t *out_value)
{
#ifdef __QNXNTO__
    InterruptLock
#else
    pthread_spin_lock
#endif
		(&output_spinlock);

    set_output_flag=true;   // aby f. obslugi przerwania wiedziala ze ma ustawic wyjscie
    binary_output=*out_value;

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
void in_out_buffer::set_input (const uint16_t binary_in_value, const uint8_t analog_in_table[])
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

    /*	analog_in_value = & read_analog;
    	binary_in_value =   & read_binary;*/

    // printf("%x\n", 0x00FF&(~odczyt));
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

    /*
    // ustawienie korzystanie z ukladu we-wy
    out8((ADR_OF_SERVO_PTR + ISA_CARD_OFFSET), IN_OUT_PTR);

    // odczytanie wejsc
    // (SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET)     0x210
    uint16_t read_analog = 0x00FF & in16((SERVO_REPLY_STATUS_ADR+ ISA_CARD_OFFSET));
    // (SERVO_REPLY_REG_1_ADR + ISA_CARD_OFFSET)       0x218
    uint16_t read_binary = 0x00FF & in16((SERVO_REPLY_REG_1_ADR + ISA_CARD_OFFSET));
    */

#ifdef __QNXNTO__
    InterruptUnlock
#else
    pthread_spin_unlock
#endif
		(&input_spinlock);

    /*	analog_in_value = & read_analog;
    	binary_in_value =   & read_binary;*/

    // printf("%x\n", 0x00FF&(~odczyt));
}


/**************************** IN_OUT_BUFFER *****************************/

} // namespace common
} // namespace edp
} // namespace mrrocpp
