#ifndef ECP_WII_GENERATOR_H
#define ECP_WII_GENERATOR_H

#include <string.h>
#include <math.h>

#include "lib/mrmath/mrmath.h"

#include "ecp/common/generator/ecp_generator.h"
#include "application/wii_teach/sensor/ecp_mp_s_wiimote.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace generator {

class wii : public common::generator::generator
{
    private:
        //ilosc krokow
	int max_steps;

	int step_no;
        //kontroler
        ecp_mp::sensor::wiimote* _wiimote;
    
    protected:
        double currentValue[6];
        double currentGripperValue;
        double requestedChange[7];
        double nextChange[7];
        double maxChange[7];
        double multipliers[7];

      	//numer kroku
        bool releasedA;
        bool rumble;
        bool stop;

        lib::Homog_matrix homog_matrix;

    public:
	/**
	 * Tworzy generator odtwarzajacy orientacje kontrolera
	 * @param zadanie
	 * @param major_axis wartosc wiekszej polosi
	 * @param minor_axis wartosc mniejszej polosi
	 * @author jedrzej
	 */
        wii (common::task::task& _ecp_task,ecp_mp::sensor::wiimote* _wiimote);

        /**
         * Generuje pierwszy krok
         * @author jedrzej
         */
        virtual bool first_step() = 0;

        /**
         * Generuje kolejne punkty wynikajace z aktualnej orientacji kontrolera
         * @author jedrzej
         */
        virtual bool next_step();

        void execute_motion(void);

        virtual void preset_position(void) = 0;

        virtual void set_position(void) = 0;

        bool calculate_change(int axis, double value);

        int get_axis(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif //ECP_WII_TEACH_GENERATOR_H
