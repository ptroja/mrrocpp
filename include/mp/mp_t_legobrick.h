// -------------------------------------------------------------------------
//                            mp_t_legobrick.h
// Definicje struktur danych i metod dla procesow MP - zadanie ukladania klockow Lego 
//  wersja z generatorami uruchaminami na poziomie ECP
// Ostatnia modyfikacja: 2008.04.02
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_LEGOBRICK_H)
#define __MP_TASK_LEGOBRICK_H

#include "mp/mp.h"
//#include "ecp_mp/ecp_mp_t_rcsc.h"
//#include "ecp_mp/ecp_mp_t_festival.h"

//#include <list>

//#include "mp/CubeState.h"
//#include "mp/SingleManipulation.h"

class mp_task_lego_brick : public mp_task
{
protected:
    // sekwencja (lista) manipulacji

    // stan kostki
    // kolory scian patrzac przez os ramienia tracka (od kolumny), w plaszczynie ziemi
    //CubeState* cube_state;

    //bool break_state;
    //bool manipulation_sequence_computed;
    // odczyt konfiguracji manipulacji
    //char* cube_initial_state;

    //bool configure_edp_force_sensor(bool configure_track, bool configure_postument);


public:
/*
    // stl'owa lista manipulacji
    std::list<SingleManipulation> manipulation_list;

    void initiate (CUBE_COLOR up_is, CUBE_COLOR down_is, CUBE_COLOR front_is,
                   CUBE_COLOR rear_is, CUBE_COLOR left_is, CUBE_COLOR right_is);
*/
    // konstruktor
    mp_task_lego_brick(configurator &_config);

    ~mp_task_lego_brick();

/*
    // MANIPULACJA
    // manipulacja pojedyncza sciana
    bool manipulate (CUBE_COLOR face_to_turn, CUBE_TURN_ANGLE turn_angle );

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
*/
    // methods for mp template
    void task_initialization(void);
    void main_task_algorithm(void);

}
; // end : class mp_task_lego_brick 

#endif
