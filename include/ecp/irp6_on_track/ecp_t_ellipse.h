#if !defined(_ECP_T_ELLIPSE_H)
#define _ECP_T_ELLIPSE_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/irp6_on_track/ecp_g_ellipse.h"
#include "ecp/common/ecp_g_smooth.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {

//limity na wartosc polosi w metrach
#define MAX_MAJOR 0.2
#define MAX_MINOR 0.1

/**
 * Kresli w powietrzu elipse o zadanych polosiach
 * @author jkurylo
 */
class ecp_task_ellipse: public common::task::ecp_task
{
protected:
	//Generator ruchu
	ecp_ellipse_generator* eg;
	common::ecp_smooth_generator* sg;
	
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
	ecp_task_ellipse(configurator &_config);
	
	/**
	 * Inicjalizuje zadanie - pobiera od uzytkownika dlugosci polosi
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
