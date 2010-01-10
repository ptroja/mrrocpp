// ------------------------------------------------------------------------
//                            int_hand.cc
//
// Funkcja obslugi przerwania -- odczyt i zapis rejestrow sprzetowych dla robota conveyor
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
#ifdef __QNXNTO__
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <hw/inout.h>
#endif

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"
#include "edp/common/edp_e_manip_and_conv.h"
#include "edp/common/hi_rydz.h"


namespace mrrocpp {
namespace edp {
namespace common {

// ------------------------------------------------------------------------
// Obsluga przerwania sprzetowego
#ifdef __QNXNTO__
const struct sigevent *
int_handler (void *arg, int int_id)
{
	common::irq_data_t *irq_data = (common::irq_data_t *) arg;
	struct sigevent & event = irq_data->event;

	return (&event);// Yoyek & wojtek
}
#endif /*__QNXNTO__ */

// ------------------------------------------------------------------------

} // namespace common
} // namespace edp
} // namespace mrrocpp
