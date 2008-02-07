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

// zbior obejmujacy mozliwe stany kostki 
enum CUBE_COLORS {UNKNOWN_CUBE_COLOR, RED, YELLOW, GREEN, BLUE, ORANGE, WHITE};
enum CUBE_TURN_ANGLE {UKNOWN_TURN_ANGLE, CL_180, CL_90, CL_0, CCL_90};

class mp_rubik_cube_solver_task_class;

CUBE_COLORS read_cube_color (char input_char);
CUBE_TURN_ANGLE read_cube_turn_angle (char input_char);


class single_manipulation_class {
public:
	CUBE_COLORS face_to_turn;
	CUBE_TURN_ANGLE turn_angle;
	
	// metody 
	void set_state (CUBE_COLORS face_to_turn_l, CUBE_TURN_ANGLE turn_angle_l);
	
	// konstruktory
	single_manipulation_class(void);
	single_manipulation_class(CUBE_COLORS face_to_turn_l, CUBE_TURN_ANGLE turn_angle_l);
	single_manipulation_class (const single_manipulation_class& cs);

};


class cube_state_class
{
protected:
	// okresla jak sciany kostki sa zorientowane wzgledem chwytaka truck'a z punkltu widzenia chwytaka
	CUBE_COLORS up, down, front, rear, left, right;
	
public:
	char cube_tab[6][9]; // NAZWA DO ZMIANY

	friend class mp_task_rubik_cube_solver;
    // konstruktory
    cube_state_class();
    cube_state_class(CUBE_COLORS up_is, CUBE_COLORS down_is, CUBE_COLORS front_is, 
		CUBE_COLORS rear_is, CUBE_COLORS left_is, CUBE_COLORS right_is);
		
	// kontruktor kopiujacy
	cube_state_class (const cube_state_class& cs);
	
	// operatory
	cube_state_class& operator= (const cube_state_class& cs);
	
	// metody
	// okresla jak sciany kostki sa zorientowane wzgledem chwytaka truck'a z punkltu widzenia chwytaka
	void set_state(CUBE_COLORS up_is, CUBE_COLORS down_is, CUBE_COLORS front_is, 
		CUBE_COLORS rear_is, CUBE_COLORS left_is, CUBE_COLORS right_is);
		
	void print_face_color(CUBE_COLORS face_name);
	void print_cube_colors();
		
};



class mp_task_rubik_cube_solver : public mp_task  
{
protected:
// sekwencja (lista) manipulacji

// stan kostki
// kolory scian patrzac przez os ramienia tracka (od kolumny), w plaszczynie ziemi
 	cube_state_class* cube_state;
 
	bool break_state;
  
    // odczyt konfiguracji manipulacji
	char* cube_initial_state;
 
 
public:

	void initiate (CUBE_COLORS up_is, CUBE_COLORS down_is, CUBE_COLORS front_is, 
		CUBE_COLORS rear_is, CUBE_COLORS left_is, CUBE_COLORS right_is);

	// stl'owa lista manipulacji
	std::list<single_manipulation_class> manipulation_list;

    // konstruktor
    mp_task_rubik_cube_solver();
	
    ~mp_task_rubik_cube_solver();
	
	
	// MANIPULACJA
	// manipulacja pojedyncza sciana
	bool manipulate(CUBE_COLORS face_to_turn, CUBE_TURN_ANGLE turn_angle );

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
