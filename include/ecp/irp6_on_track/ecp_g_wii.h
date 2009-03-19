#if !defined_ecp_wii_generator_H
# define _ecp_wii_generator_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"

#define PI 3.141592

class ecp_wii_generator : public ecp_generator
{
	private:
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
	 * Tworzy generator odtwarzajacy orientacje kontrolera
	 * @param zadanie
	 * @param major_axis wartosc wiekszej polosi
	 * @param minor_axis wartosc mniejszej polosi
	 * @author jedrzej
	 */
    ecp_wii_generator (ecp_task& _ecp_task);

    /**
     * Generuje pierwszy krok
     * @author jedrzej
     */
    virtual bool first_step();

    /**
     * Generuje kolejne punkty wynikajace z aktualnej orientacji kontrolera
     * @author jedrzej
     */
    virtual bool next_step();

    double* getFirstPosition();
};

#endif
