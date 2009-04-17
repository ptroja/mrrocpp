#if !defined_ecp_ellipse_generator_H
# define _ecp_ellipse_generator_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

#define PI 3.141592

class ellipse : public common::generator::base
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
    ellipse (common::task::task& _ecp_task,double major_axis,double minor_axis,int max_steps);
    
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
    
    double* getFirstPosition();
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
