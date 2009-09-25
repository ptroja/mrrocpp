// ------------------------------------------------------------------------
//                            int_hand.cc
//
// Funkcja obslugi przerwania -- odczyt i zapis rejestrow sprzetowych dla robota irp6 postument
//
// Ostatnia modyfikacja: 2005
// ------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <hw/inout.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"


// Klasa edp_irp6m_effector.
#include "edp/polycrank/edp_e_polycrank.h"
// Klasa hardware_interface.
#include "edp/polycrank/hi_polycrank.h"


namespace mrrocpp {
namespace edp {
namespace common {

extern polycrank::effector* master;   // Bufor polecen i odpowiedzi EDP_MASTER

}
}
}

namespace mrrocpp {
namespace edp {
namespace polycrank {

// Zmienne globalne do komunikacji z procedura obslugi przerwan

extern struct sigevent event; // by y&w
extern volatile common::motor_data md; // Aktualne dane we/wy (obsluga przerwania)



// ------------------------------------------------------------------------

// Obsluga przerwania sprzetowego

const struct sigevent *
int_handler (void *arg, int int_id)
{
	common::status_of_a_dof robot_status[POLYCRANK_NUM_OF_SERVOS];
	short int low_word, high_word;
	int i;

	md.hardware_error = (uint64_t) lib::ALL_RIGHT; // Nie ma bledow sprzetowych

	if(common::master->test_mode)
	{
		return (&event); // by Y&W
	}

	return (&event);

}

// ------------------------------------------------------------------------

} // namespace common
} // namespace edp
} // namespace mrrocpp
