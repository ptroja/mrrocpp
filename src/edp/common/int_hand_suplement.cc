#ifdef __QNXNTO__
#include <sys/siginfo.h>
#endif

#include "edp/common/edp_e_motor_driven.h"
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
