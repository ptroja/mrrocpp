#if !defined_ecp_wii_teach_generator_H
# define _ecp_wii_teach_generator_H

#include <string.h>
#include <math.h>

#include "ecp/common/ecp_generator.h"
#include "ecp_mp/sensor/ecp_mp_s_wiimote.h"

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
        ecp_mp::sensor::wiimote* _wiimote;
        bool releasedA;
        bool rumble;
        bool stop;

        double currentChange[7];
        double requestedChange[7];
        double nextChange[7];
        double maxChange[7];
        double multipliers[7];

    public:
	/**
	 * Tworzy generator odtwarzajacy orientacje kontrolera
	 * @param zadanie
	 * @param major_axis wartosc wiekszej polosi
	 * @param minor_axis wartosc mniejszej polosi
	 * @author jedrzej
	 */
        wii_teach (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote);

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

        void set_position(void);

        bool calculate_position(void);

        int get_axis(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
