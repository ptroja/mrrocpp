#if !defined(_ECP_T_WII_VELOCITY_H)
#define _ECP_T_WII_VELOCITY_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/irp6_on_track/ecp_g_wii_velocity.h"
#include "ecp/common/ecp_g_smooth.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

/**
 * Odtwarza orientacje kontrolera
 * @author jkurylo
 */
class wii_velocity: public common::task::task
{
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

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
