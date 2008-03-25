#ifndef ECP_G_TZU_H_
#define ECP_G_TZU_H_

#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/common/ecp_teach_in_generator.h"
#include "lib/mathtr.h"


// Generator do wykrywania zmiany wagi chwytaka wraz z obiektem chwytanym
// ciezary wyskalowane w newtonach
#define WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE 10
#define USLEEP_TIME 10000

class force_meassure_generator : public ecp_generator
{
private:
    double weight_difference;  // roznica wagi do wykrycia
    double weight_in_cyclic_buffer[WEIGHT_MEASSURE_GENERATOR_BUFFER_SIZE];
    int current_buffer_pointer;
    double initial_weight; // pierwszy zmierzony ciezar
    bool initial_weight_counted; // czy wyznaczono juz poczatkowy ciezar
    int catch_lag; // ilosc potwierdzen zmiany masy do zwrocenia false;
    int initial_catch_lag; // ilosc potwierdzen zmiany masy do zwrocenia false;
    double catch_time; // czas przez ktory ma byc stwierdzona zmian ciê¿aru
    bool terminate_state_recognized; // wykryto warunek koncowy
	int what_to_meassure;

    // wstawienie elementu do bufora cyklicznego
    void insert_in_buffer(const double fx);

    // wyznaczenie sredniej arytmetycznej ciezarow zapisanych w buforze
    double check_average_weight_in_buffer(void) const;

    // czyszczenie bufora cyklicznego
    void clear_buffer();

public:

    // ustawia nowa roznice wag
    void set_weight_difference(const double _weight_difference);

    // konstruktor
    force_meassure_generator(ecp_task& _ecp_task, double _weight_difference=0.0, double _catch_time = 1.0, int what_to_meassure = 2);
	
	void change_meassurement(int what);
	double get_meassurement();
	
    bool first_step ();
    bool next_step ();

}
; // end:


#endif /*ECP_G_TZU_H_*/
