#if !defined_ecp_wii_generator_H
# define _ecp_wii_generator_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

#define PI 3.141592

class wii : public common::generator::base
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
    wii (common::task::base& _ecp_task);

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

    void execute_motion(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
