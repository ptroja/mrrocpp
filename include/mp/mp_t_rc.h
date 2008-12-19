// -------------------------------------------------------------------------
//                            mp_task_rc.h
// Definicje struktur danych i metod dla procesow MP - zadanie ukladania kostki Rubika
// 
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_RC_H)
#define __MP_TASK_RC_H

#include "mp/mp.h"
#include <list>

#include "mp/CubeState.h"
#include "mp/SingleManipulation.h"

class mp_task_rubik_cube_solver : public mp_task  
{
protected:
// sekwencja (lista) manipulacji

// stan kostki
// kolory scian patrzac przez os ramienia tracka (od kolumny), w plaszczynie ziemi
 	CubeState* cube_state;
 
  
    // odczyt konfiguracji manipulacji
	char* cube_initial_state;
 
 
public:

	// stl'owa lista manipulacji
	std::list<SingleManipulation> manipulation_list;

	void initiate (CUBE_COLOR up_is, CUBE_COLOR down_is, CUBE_COLOR front_is, 
		CUBE_COLOR rear_is, CUBE_COLOR left_is, CUBE_COLOR right_is);

    // konstruktor
    mp_task_rubik_cube_solver(configurator &_config);
	
    ~mp_task_rubik_cube_solver();

	
	// MANIPULACJA
	// manipulacja pojedyncza sciana
	void manipulate (CUBE_COLOR face_to_turn, CUBE_TURN_ANGLE turn_angle );

	// wykonanie sekwecji manipulacji poszczegolnymi scianami
	void execute_manipulation_sequence();
	
	//wykonanie sekwencji manipulacji w celu identyfikacji kolorow
	void identify_colors();
	
	bool communicate_with_windows_solver();
	
	// OPERACJE
	
	// obrot sciany
	void face_turn_op (CUBE_TURN_ANGLE turn_angle);
	// zmiana sciany (przelozenie kostki)
	void face_change_op (CUBE_TURN_ANGLE turn_angle);
	// dojscie
	void approach_op (int mode);
	// odejscie
	void departure_op ();


	// METODY POMOCNICZE
	
	// rozwieranie chwytakow
	void gripper_opening(double track_increment, double postument_increment, int motion_time);
	
	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);

}; // end : class MP_nose_run_force_generator

#endif
