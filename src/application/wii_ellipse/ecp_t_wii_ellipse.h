#ifndef ECP_T_WII_ELLIPSE_H
#define ECP_T_WII_ELLIPSE_H

#include "ecp_mp/task/ecp_mp_task.h"
#include "application/wii_ellipse/generator/ecp_g_wii_ellipse.h"
#include "ecp/common/generator/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

/** @defgroup wii_ellipse Wii ellipsis
 *  @ingroup application
 *
 *  Moves irp6ot on elliptical trajectory
 *  using input from wii-mote controller and saves
 *
 *  @{
 */

//limity na wartosc polosi w metrach
#define MAX_MAJOR 0.2
#define MAX_MINOR 0.1

/**
 * Kresli w powietrzu elipse o zadanych polosiach
 * @author jkurylo
 */
class wii_ellipse: public common::task::task
{
protected:
	//Generator ruchu
	generator::wii_ellipse* eg;
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
	wii_ellipse(lib::configurator &_config);

	/**
	 * Realizuje zadanie
	 * @author jkurylo
	 */
	void main_task_algorithm(void);
};

/** @} */ // end of wii_ellipse

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif //ECP_T_WII_ELLIPSE_H
