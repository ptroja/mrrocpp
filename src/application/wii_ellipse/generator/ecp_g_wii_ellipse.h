#ifndef ECP_WII_ELLIPSE_GENERATOR_H
#define ECP_WII_ELLIPSE_GENERATOR_H

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

/** @addtogroup wii_ellipse
 *
 *  @{
 */

class wii_ellipse : public common::generator::generator
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
    wii_ellipse (common::task::task& _ecp_task,double major_axis,double minor_axis,int max_steps);

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

/** @} */ // end of wii_ellipse

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif //ECP_WII_ELLIPSE_GENERATOR_H
