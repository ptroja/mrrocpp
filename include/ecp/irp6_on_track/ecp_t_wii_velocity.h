#if !defined(_ECP_T_WII_VELOCITY_H)
#define _ECP_T_WII_VELOCITY_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/irp6_on_track/ecp_g_wii_velocity.h"
#include "ecp/common/ecp_g_smooth.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot {

/**
 * Odtwarza orientacje kontrolera
 * @author jkurylo
 */
class ecp_task_wii_velocity: public common::task::ecp_task
{
protected:
//	ecp_wii_velocity_generator* eg;
	common::generator::ecp_tff_nose_run_generator* eg;
public:
	/**
	 * Tworzy obiekt zadania
	 * @param _config konfigurator
	 * @author jedrzej
	 */
	ecp_task_wii_velocity(configurator &_config);

	/**
	 * Inicjalizuje zadanie
	 * @author jkurylo
	 */
	void task_initialization(void);

	/**
	 * Realizuje zadanie
	 * @author jkurylo
	 */
	void main_task_algorithm(void);
};


} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
