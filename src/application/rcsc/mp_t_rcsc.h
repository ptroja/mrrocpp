// -------------------------------------------------------------------------
//                            mp_task_rcsc.h
// Definicje struktur danych i metod dla procesow MP - zadanie ukladania kostki Rubika
//  wersja z generatorami uruchaminami na poziomie ECP
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_RCSC_H)
#define __MP_TASK_RCSC_H

#include "application/rcsc/ecp_mp_t_rcsc.h"
#include "robot/festival/ecp_mp_t_festival.h"

#include <list>

#include "CubeState.h"
#include "SingleManipulation.h"

namespace mrrocpp {
namespace mp {
namespace task {

class rubik_cube_solver : public task
{
protected:
	int vis_servoing;
	// sekwencja (lista) manipulacji

	// stan kostki
	// kolory scian patrzac przez os ramienia tracka (od kolumny), w plaszczynie ziemi
	common::CubeState* cube_state;

	bool manipulation_sequence_computed;
	// odczyt konfiguracji manipulacji
	char* cube_initial_state;

	void configure_edp_force_sensor(bool configure_track, bool configure_postument);

public:
	/// utworzenie robotow
	void create_robots(void);
	// stl'owa lista manipulacji
	std::list <common::SingleManipulation> manipulation_list;

	void
			initiate(common::CUBE_COLOR up_is, common::CUBE_COLOR down_is, common::CUBE_COLOR front_is, common::CUBE_COLOR rear_is, common::CUBE_COLOR left_is, common::CUBE_COLOR right_is);

	// konstruktor
	rubik_cube_solver(lib::configurator &_config);

	~rubik_cube_solver();

	// MANIPULACJA
	// manipulacja pojedyncza sciana
	void manipulate(common::CUBE_COLOR face_to_turn, common::CUBE_TURN_ANGLE turn_angle);

	// wykonanie sekwecji manipulacji poszczegolnymi scianami
	void execute_manipulation_sequence();

	//wykonanie sekwencji manipulacji w celu identyfikacji kolorow
	void identify_colors();

	bool communicate_with_windows_solver();

	// OPERACJE

	// obrot sciany
	void face_turn_op(common::CUBE_TURN_ANGLE turn_angle);
	// zmiana sciany (przelozenie kostki)
	void face_change_op(common::CUBE_TURN_ANGLE turn_angle);
	// dojscie
	void approach_op(int mode);
	// odejscie
	void departure_op();

	// METODY POMOCNICZE

	// rozwieranie chwytakow
	void gripper_opening(double track_increment, double postument_increment, int motion_time);

	// methods for mp template
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
