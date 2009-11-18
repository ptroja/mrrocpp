#if !defined_ecp_wii_teach_generator_H
# define _ecp_wii_teach_generator_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"

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
        //kontroler
        lib::sensor* _wiimote;
        int releasedA;

    public:
	/**
	 * Tworzy generator odtwarzajacy orientacje kontrolera
	 * @param zadanie
	 * @param major_axis wartosc wiekszej polosi
	 * @param minor_axis wartosc mniejszej polosi
	 * @author jedrzej
	 */
        wii_teach (common::task::task& _ecp_task,lib::sensor* _wiimote);

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

        void clear_position(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
