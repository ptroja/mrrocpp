#if !defined_ecp_wii_teach_generator_H
# define _ecp_wii_teach_generator_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"
#include "ecp/common/ecp_g_smooth2.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

class wii_teach : public common::generator::generator
{
    private:
        //ilosc krokow
	int max_steps;
	//numer kroku
	int step_no;
	//nowa pozycja
    	double position[8];
        //kontroler
        lib::sensor* _wiimote;
        common::generator::smooth2* sg;

    public:
	/**
	 * Tworzy generator odtwarzajacy orientacje kontrolera
	 * @param zadanie
	 * @param major_axis wartosc wiekszej polosi
	 * @param minor_axis wartosc mniejszej polosi
	 * @author jedrzej
	 */
        wii_teach (common::task::task& _ecp_task,lib::sensor* _wiimote,common::generator::smooth2*);

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

        void execute_motion(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
