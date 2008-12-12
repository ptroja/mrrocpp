#if !defined_ECP_G_ELLIPSE_H
# define _ECP_G_ELLIPSE_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"

#define PI 3.141592

class ecp_g_ellipse : public ecp_generator
{
	private:
		//wieksza polos
		double major_axis;
		//mniejsza polos
		double minor_axis;
		//ilosc krokow
		int max_steps;
		//przyrost kata
		double d_rad;
		//wartosc kata
		double rad;
		//numer kroku
		int step_no;
		//nowa pozycja
    	double position[8];

public:
	/**
	 * Tworzy generator elipsy
	 * @param zadanie
	 * @param major_axis wartosc wiekszej polosi
	 * @param minor_axis wartosc mniejszej polosi
	 * @param max_steps ilosc krokow, w ktorych wykonany ma byc ruch po elipsie
	 * @author jedrzej
	 */
    ecp_g_ellipse (ecp_task& _ecp_task,double major_axis,double minor_axis,int max_steps);
    
    /**
     * Generuje pierwszy krok - ustawienie w pozycji (0,b)
     * @author jedrzej
     */
    virtual bool first_step();

    /**
     * Generuje kolejne punkty trajektorii
     * @author jedrzej
     */    
    virtual bool next_step();
};

#endif
