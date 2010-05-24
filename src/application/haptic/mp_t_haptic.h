// -------------------------------------------------------------------------
//                            task/mp_t_haptic.h
//
// MP task for two robot haptic device
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_T_HAPTIC_H)
#define __MP_T_HAPTIC_H

#include "mp/mp.h"

namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup haptic Haptic coupling
 *  @ingroup application
 *  A two robot application of IRp6 manipulator's in haptic coupling
 *  @{
 */

class haptic : public task
{
protected:

	void configure_edp_force_sensor(bool configure_track, bool configure_postument);

public:

	haptic(lib::configurator &_config);

	// methods for mp template
	void main_task_algorithm(void);

};

/** @} */

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
