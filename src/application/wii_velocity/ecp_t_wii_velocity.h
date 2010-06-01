#ifndef ECP_T_WII_VELOCITY_H
#define ECP_T_WII_VELOCITY_H

#include "ecp_mp/task/ecp_mp_task.h"
#include "application/wii_velocity/generator/ecp_g_wii_velocity.h"
#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

/** @defgroup wii_velocity Wii velocity
 *  @ingroup application
 *
 *  Moves irp6ot by increasing/decreasing joint values
 *  using input from wii-mote controller and saves
 *
 *  @{
 */

/**
 * Odtwarza orientacje kontrolera
 * @author jkurylo
 */
class wii_velocity: public common::task::task {
protected:
	//	ecp_wii_velocity_generator* eg;
	common::generator::tff_nose_run* eg;
public:
	/**
	 * Tworzy obiekt zadania
	 * @param _config konfigurator
	 * @author jedrzej
	 */
	wii_velocity(lib::configurator &_config);

	/**
	 * Realizuje zadanie
	 * @author jkurylo
	 */
	void main_task_algorithm(void);
};

/** @} */// end of wii_velocity

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif //ECP_T_WII_VELOCITY_H
