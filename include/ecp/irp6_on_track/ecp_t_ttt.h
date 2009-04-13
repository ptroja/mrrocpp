#if !defined(_ECP_T_TTT_H)
#define _ECP_T_TTT_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

/**
 * Kresli w powietrzu siatke jak do gry w kolko-i-krzyzyk
 * @author jkurylo
 */
class ecp_task_ttt: public common::task::ecp_task
{
protected:
	//Generator ruchu
	common::generator::ecp_smooth_generator* sg;

public:
	/**
	 * Tworzy obiekt zadania
	 * @param _config konfigurator
	 * @author jedrzej
	 */
	ecp_task_ttt(configurator &_config);
	
	/**
	 * Inicjalizuje zadanie - wczytuje trajektorie z pliku
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
