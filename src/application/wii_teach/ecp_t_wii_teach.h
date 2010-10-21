#ifndef ECP_T_WII_TEACH_H
#define ECP_T_WII_TEACH_H

#include "base/ecp_mp/ecp_mp_task.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "application/wii_teach/generator/ecp_g_wii_relative.h"
#include "application/wii_teach/generator/ecp_g_wii_absolute.h"
#include "application/wii_teach/generator/ecp_g_wii_joint.h"
#include "application/wii_teach/sensor/ecp_mp_s_wiimote.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {


/** @defgroup wii_teach Wii teach
 *  @ingroup application
 *
 *  Moves irp6ot using input from wii-mote controller and saves
 *  the generated trajectory
 *
 *  @{
 */

/**
 * @author jkurylo
 */
class wii_teach: public common::task::task
{
    protected:
	//Generator ruchu
        common::generator::newsmooth* sg;
        irp6ot_m::generator::wii_absolute* ag;
        irp6ot_m::generator::wii_relative* rg;
        irp6ot_m::generator::wii_joint* jg;
        ecp_mp::sensor::wiimote_t lastButtons;
        ecp_mp::sensor::wiimote_t buttonsPressed;
        char path[80];
        char filename[20];
        std::vector <double> coordinates;

        lib::Homog_matrix homog_matrix;

        class n;
        class n
        {
            public:
                n* next;
                n* prev;
                int id;
                double position[6];

                n() : next(NULL), prev(NULL) {}

        };

        typedef n node;

        struct
        {
            node* head;
            node* tail;
            int count;
            node* current;
            int position;
        } trajectory;

        void updateButtonsPressed();

    public:
	/**
	 * Tworzy obiekt zadania
	 * @param _config konfigurator
	 * @author jedrzej
	 */
	wii_teach(lib::configurator &_config);

	/**
	 * Realizuje zadanie
	 * @author jkurylo
	 */
	void main_task_algorithm(void);

        void print_trajectory(void);

        void move_to_current(void);

        bool get_filenames(void);

        int load_trajectory(void);

        void save_trajectory(void);
};

/** @} */ // end of wii_teach

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


#endif //ECP_T_WII_TEACH_H
