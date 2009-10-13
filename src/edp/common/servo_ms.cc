//
//                         SERVO GROUP Process -- main
//
// Ostatnia modyfikacja:  2005 by Y
// -------------------------------------------------------------------

#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "edp/common/hi_rydz.h"
#include "edp/common/servo_gr.h"
#include "edp/common/edp.h"
#include "lib/mis_fun.h"

#include <memory>

namespace mrrocpp {
namespace edp {
namespace common {

void * manip_and_conv_effector::servo_thread_start(void* arg)
{
    return static_cast<manip_and_conv_effector*> (arg)->servo_thread(arg);
}

void * manip_and_conv_effector::servo_thread(void* arg)
{
	// servo buffer has to be created before servo thread starts
	std::auto_ptr<servo_buffer> sb(return_created_servo_buffer(*this)); // bufor do komunikacji z EDP_MASTER

    lib::set_thread_priority(pthread_self(), MAX_PRIORITY+2);

    /* BEGIN SERVO_GROUP */

    for (;;)
    {
        // komunikacja z transformation
        if (!sb->get_command())
        {

            rb_obj.lock_mutex();
            rb_obj.step_data.servo_mode = false; // tryb bierny
            rb_obj.unlock_mutex();

            /* Nie otrzymano nowego polecenia */
            /* Krok bierny - zerowy przyrost polozenia */
            // Wykonanie pojedynczego kroku ruchu
            sb->Move_passive();
        }
        else
        {
        	// nowe polecenie
            rb_obj.lock_mutex();
            rb_obj.step_data.servo_mode = true; // tryb czynny
            rb_obj.unlock_mutex();

            switch (sb->command_type())
            {
				case lib::SYNCHRONISE:
					sb->synchronise(); // synchronizacja
					break;
				case lib::MOVE:
					sb->Move(); // realizacja makrokroku ruchu
					break;
				case lib::READ:
					sb->Read(); // Odczyt polozen
					break;
				case lib::SERVO_ALGORITHM_AND_PARAMETERS:
					sb->Change_algorithm(); // Zmiana algorytmu serworegulacji lub jego parametrow
					break;
				default:
					// niezidentyfikowane polecenie (blad) nie moze wystapic, bo juz
					// wczesniej zostalo wychwycone przez get_command()
					break;
            }
        } // end: else
    }

    return NULL;
} // end: main() SERVO_GROUP


/*---------------------------------------------------------------------------*/

} // namespace common
} // namespace edp
} // namespace mrrocpp
