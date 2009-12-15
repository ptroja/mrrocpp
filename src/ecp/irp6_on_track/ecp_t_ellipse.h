#if !defined(_ECP_T_ELLIPSE_H)
#define _ECP_T_ELLIPSE_H

#include "ecp_mp/task/ecp_mp_task.h"
#include "ecp/irp6_on_track/ecp_g_ellipse.h"
#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

//limity na wartosc polosi w metrach
#define MAX_MAJOR 0.2
#define MAX_MINOR 0.1

/**
 * Kresli w powietrzu elipse o zadanych polosiach
 * @author jkurylo
 */
class ellipse: public common::task::task
{
protected:
	//Generator ruchu
	generator::ellipse* eg;
	common::generator::smooth* sg;

	/**
	 * Pobiera od uzytkownika wartosc typu double
	 * @param name nazwa parametru
	 * @param min minimalna wartosc parametru
	 * @param max maksymalna wartosc parametru
	 * @return pobrana wartosc parametru
	 * @author jedrzej
	 */
	double read_double(char * name,double min,double max);

public:
	/**
	 * Tworzy obiekt zadania
	 * @param _config konfigurator
	 * @author jedrzej
	 */
	ellipse(lib::configurator &_config);

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
