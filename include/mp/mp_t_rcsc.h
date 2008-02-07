// -------------------------------------------------------------------------
//                            mp_task_rcsc.h
// Definicje struktur danych i metod dla procesow MP - zadanie ukladania kostki Rubika
//  wersja z generatorami uruchaminami na poziomie ECP
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_RCSC_H)
#define __MP_TASK_RCSC_H

#include "mp/mp.h"
#include "ecp_mp/ecp_mp_t_rcsc.h"

#include "mp/CubeState.h"
#include "mp/SingleManipulation.h"

#include <list>

class mp_rubik_cube_solver_task_class;


class mp_task_rubik_cube_solver : public mp_task  
{
protected:
// sekwencja (lista) manipulacji

// stan kostki
// kolory scian patrzac przez os ramienia tracka (od kolumny), w plaszczynie ziemi
 	CubeState* cube_state;
 
	bool break_state;
  	bool manipulation_sequence_computed;
    // odczyt konfiguracji manipulacji
	char* cube_initial_state;
 
 
public:

	// stl'owa lista manipulacji
	std::list<SingleManipulation> manipulation_list;

	void initiate (CUBE_COLORS up_is, CUBE_COLORS down_is, CUBE_COLORS front_is, 
		CUBE_COLORS rear_is, CUBE_COLORS left_is, CUBE_COLORS right_is);

    // konstruktor
    mp_task_rubik_cube_solver();
	
    ~mp_task_rubik_cube_solver();

	
	// MANIPULACJA
	// manipulacja pojedyncza sciana
	bool manipulate (CUBE_COLORS face_to_turn, CUBE_TURN_ANGLE turn_angle );

	// wykonanie sekwecji manipulacji poszczegolnymi scianami
	bool execute_manipulation_sequence();
	
	//wykonanie sekwencji manipulacji w celu identyfikacji kolorow
	bool identify_colors();
	
	bool communicate_with_windows_solver();
	
	// OPERACJE
	
	// obrot sciany
	bool face_turn_op (CUBE_TURN_ANGLE turn_angle);
	// zmiana sciany (przelozenie kostki)
	bool face_change_op (CUBE_TURN_ANGLE turn_angle);
	// dojscie
	bool approach_op (int mode);
	// odejscie
	bool departure_op ();


	// METODY POMOCNICZE
	
	// rozwieranie chwytakow
	bool gripper_opening(double track_increment, double postument_increment, int motion_time);
	
	// methods for mp template
	void task_initialization(void);
	void main_task_algorithm(void);

}; // end : class MP_nose_run_force_generator

#endif
