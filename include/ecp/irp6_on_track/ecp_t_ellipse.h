#if !defined(_ECP_T_ELLIPSE_H)
#define _ECP_T_ELLIPSE_H

#include "ecp_mp/ecp_mp_task.h"
#include "ecp/common/ecp_g_smooth.h"

//limity na wartosc polosi w metrach
#define MAX_MAJOR 0.2
#define MAX_MINOR 0.1

/**
 * Kresli w powietrzu elipse o zadanych polosiach
 * @author jkurylo
 */
class ecp_task_ellipse: public ecp_task
{
protected:
	//Generator ruchu
	ecp_smooth_generator* sg;
	
	/**
	 * Pobiera od uzytkownika wartosc polosi
	 * @param name nazwa polosi
	 * @param limit maksymalna wartosc polosi
	 * @return wartosc polosi
	 * @author jedrzej
	 */
	double read_axis(char * name,double limit);	

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


#endif
