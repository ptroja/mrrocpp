// -------------------------------------------------------------------------
//                            mp_t_legobrick.h
// Definicje struktur danych i metod dla procesow MP - zadanie ukladania klockow Lego 
//  wersja z generatorami uruchaminami na poziomie ECP
// Ostatnia modyfikacja: 2008.04.02
// -------------------------------------------------------------------------

#if !defined(__MP_TASK_LEGOBRICK_H)
#define __MP_TASK_LEGOBRICK_H

#include "mp/mp.h"


class mp_task_lego_brick : public mp_task
{
protected:



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

    void task_initialization(void);
    void main_task_algorithm(void);

}
; // end : class mp_task_lego_brick 

#endif
