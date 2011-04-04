#include <cstring>
#include <stdint.h>

#include <pthread.h>

#include "base/edp/in_out.h"

namespace mrrocpp {
namespace edp {
namespace common {

in_out_buffer::in_out_buffer()
{
	binary_input = 0;
	binary_output = 0;
	for (int i = 0; i < 8; i++) {
		analog_input[i] = 0;
	}

	// inicjacja spinlockow
#if (defined(__APPLE__) && defined(__MACH__))
	input_spinlock = 0;
	output_spinlock = 0;
#else
	pthread_spin_init(&input_spinlock, PTHREAD_PROCESS_PRIVATE);
	pthread_spin_init(&output_spinlock, PTHREAD_PROCESS_PRIVATE);
#endif
	set_output_flag = false;
}

in_out_buffer::~in_out_buffer()
{
#if !(defined(__APPLE__) && defined(__MACH__))
	pthread_spin_destroy(&input_spinlock);
	pthread_spin_destroy(&output_spinlock);
#endif
}

// ustawienie wyjsc
void in_out_buffer::set_output(uint16_t out_value)
{
	SPIN_LOCK (&output_spinlock);

	set_output_flag = true; // aby f. obslugi przerwania wiedziala ze ma ustawic wyjscie
	binary_output = out_value;

	SPIN_UNLOCK (&output_spinlock);
}

// odczytanie wyjsc
void in_out_buffer::get_output(uint16_t *out_value)
{
	SPIN_LOCK (&output_spinlock);

	*out_value = binary_output;

	SPIN_UNLOCK (&output_spinlock);
}

// ustawienie wejsc
void in_out_buffer::set_input(uint16_t binary_in_value, const uint8_t analog_in_table[])
{
	SPIN_LOCK (&input_spinlock);

	binary_input = binary_in_value; // wejscie binarne
	for (int i = 0; i < 8; i++) {
		analog_input[i] = analog_in_table[i];
	}

	SPIN_UNLOCK (&input_spinlock);
}

// odczytanie wejsc
void in_out_buffer::get_input(uint16_t *binary_in_value, uint8_t analog_in_table[])
{
	SPIN_LOCK (&input_spinlock);

	*binary_in_value = binary_input; // wejscie binarne
	for (int i = 0; i < 8; i++) {
		analog_in_table[i] = analog_input[i];
	}

	SPIN_UNLOCK (&input_spinlock);
}

} // namespace common
} // namespace edp
} // namespace mrrocpp
