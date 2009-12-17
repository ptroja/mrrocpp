#ifndef ECP_T_WII_VELOCITY_H
#define ECP_T_WII_VELOCITY_H

#include "ecp_mp/task/ecp_mp_task.h"
#include "task/wii_velocity/generator/ecp_g_wii_velocity.h"
#include "ecp/common/generator/ecp_g_smooth2.h"


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

#endif //ECP_T_WII_VELOCITY_H
