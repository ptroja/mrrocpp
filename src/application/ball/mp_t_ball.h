// -------------------------------------------------------------------------
//                            task/mp_t_ball.h
//
// MP task for two robot ball device
// Ostatnia modyfikacja: 2007
// -------------------------------------------------------------------------

#if !defined(__MP_T_HAPTIC_H)
#define __MP_T_HAPTIC_H

#include "base/mp/mp_task.h"

namespace mrrocpp {
namespace mp {
namespace task {

/** @defgroup ball Cooperated object transport
 *  @ingroup application
 *  A two robot application of IRp6 manipulator's in ball transport
 *  @{
 */

class ball : public task
{
private:
	void configure_edp_force_sensor(bool configure_track, bool configure_postument);

public:
	ball(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);

	// methods for mp template
	void main_task_algorithm(void);
};

/** @} */ // end of ball

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
