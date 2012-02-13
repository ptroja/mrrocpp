#ifndef ECP_T_WII_TEACH_H
#define ECP_T_WII_TEACH_H

#include "base/ecp_mp/ecp_mp_task.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"
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
	//Generatory ruchu
        common::generator::newsmooth* sg;
        irp6ot_m::generator::wii_absolute* ag;
        irp6ot_m::generator::wii_relative* rg;
        irp6ot_m::generator::wii_joint* jg;
    
	common::generator::get_position* gg;
    
	//Przyciski
        ecp_mp::sensor::wiimote_t lastButtons;
        ecp_mp::sensor::wiimote_t buttonsPressed;
    
	//Plik z trajektoria
        char path[80];
        char filename[40];
    
        std::vector <double> coordinates;

        lib::Homog_matrix homog_matrix;

	//Numeracja wezlow
	int cnt;
    
	//Numer aktualnego generatora
	int gen;
	
	//Czy wybrano plik trajektorii
	bool has_filenames;
	
	//Aktualny generator
	common::generator::generator* g;
    
	//Bufor na komunikaty konsolowe
	char buffer[200];
	
	//Struktura komunikacyjna z Wii-mote
	ecp_mp::sensor::wii_command_t message;
    
	lib::ECP_POSE_SPECIFICATION pose_spec;
	int axis_num;
	
	std::string velocity;
	std::string acceleration;
	std::string type;
	std::string mode;
    
        class n;
        class n
        {
            public:
                n* next;
                n* prev;
                int id;
		std::vector<double> position;

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

	//Odtwarzanie trajektorii
        void move_to_current(void);
	void move_to_next(void);
	void move_to_prev(void);
	void move_to_first(void);
	void move_to_last(void);
	
	//Obsluga przyciskow
	void handle12();
	void handleA();
	void handleB();
	void handlePlus();
	void handleMinus();
	void handleHome();

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
