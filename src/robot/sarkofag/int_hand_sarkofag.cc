// ------------------------------------------------------------------------
//                            int_hand.cc
//
// Funkcja obslugi przerwania -- odczyt i zapis rejestrow sprzetowych dla robota sarkofag
//
// Ostatnia modyfikacja: 2005
// ------------------------------------------------------------------------

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <cctype>
#include <sys/wait.h>
#include <sys/types.h>
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <hw/inout.h>
#endif

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

// Klasa edp_sarkofag_effector.
#include "robot/sarkofag/edp_e_sarkofag.h"
// Klasa hardware_interface.
#include "robot/hi_moxa/hi_moxa.h"

namespace mrrocpp {
namespace edp {
namespace common {

extern sarkofag::effector* master; // Bufor polecen i odpowiedzi EDP_MASTER

}

namespace sarkofag {

// ------------------------------------------------------------------------

// Obsluga przerwania sprzetowego

const struct sigevent *
int_handler(void *arg, int int_id)
{
}

// ------------------------------------------------------------------------

} // namespace common
} // namespace edp
} // namespace mrrocpp
