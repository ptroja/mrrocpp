#if !defined(_ECP_T_WII_H)
#define _ECP_T_WII_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/irp6_on_track/ecp_g_wii.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {


/**
 * Odtwarza orientacje kontrolera
 * @author jkurylo
 */
class ecp_task_wii: public common::task::base
{
protected:
	//Generator ruchu
	generator::ecp_wii_generator* eg;
	common::generator::smooth* sg;

public:
	/**
	 * Tworzy obiekt zadania
	 * @param _config konfigurator
	 * @author jedrzej
	 */
	ecp_task_wii(configurator &_config);

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

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif
