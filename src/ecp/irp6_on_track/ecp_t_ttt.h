#if !defined(_ECP_T_TTT_H)
#define _ECP_T_TTT_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

/**
 * Kresli w powietrzu siatke jak do gry w kolko-i-krzyzyk
 * @author jkurylo
 */
class ttt: public common::task::task
{
protected:
	//Generator ruchu
	common::generator::smooth* sg;

public:
	/**
	 * Tworzy obiekt zadania
	 * @param _config konfigurator
	 * @author jedrzej
	 */
	ttt(lib::configurator &_config);

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
