#ifndef MP_GENERATOR_H_
#define MP_GENERATOR_H_

#include "mp/mp_robot.h"
#include "mp/mp_task.h"

#include <map>

class mp_generator {
    // Klasa bazowa dla generatorow trajektorii (klasa abstrakcyjna)
    // Sluzy zarowno do wyznaczania nastepnej wartosci zadanej jak i
    // sprawdzania spelnienia warunku koncowego

protected:
	sr_ecp* sr_ecp_msg;    // by Y - Wskaznik na obiekt do komunikacji z SR

	int idle_step_counter; // Licznik jalowych krokow sterowania (bez wykonywania ruchu)

public:

	// zbior obejmujacy mozliwe stany obiektu klasy generator_set
	enum GEN_SET_PHASE {
		BEFORE_FIRST_STEP,
		AFTER_STEP,
		AFTER_INITIATE_READING,
		WAITING_FOR_ECP_BEFORE_EXECUTE_MOTION,
		AFTER_EXECUTE_MOTION,
		AFTER_GET_READING,
		GS_FINISHED
	};

	GEN_SET_PHASE phase; // faza w ktorej znajduje sie generator
	bool new_pulse_checked; // czy nowy puls zostal sprawdzony (wykorzystywane w scheduller_run() )
	bool wait_for_ECP_pulse; // okresla czy przed next step move ma sie zawieszac w oczekwianiu na puls z ECP
		// wykorzystywane przy luznej i sporadycznej wspolpracy robotow.
	
	mp_task& mp_t;

	bool trigger; // informacja czy pszyszedl puls trigger

	// mapa wszystkich czujnikow
	std::map <SENSOR_ENUM, sensor*> sensor_m;

	// mapa wszystkich transmiterow
	std::map <TRANSMITTER_ENUM, transmitter*> transmitter_m;

	// mapa wszystkich robotow
	std::map <ROBOT_ENUM, mp_robot*> robot_m;

	mp_generator(mp_task& _mp_task);

	virtual ~mp_generator(void);
	virtual bool first_step (void) = 0;
	// generuje pierwszy krok ruchu -
	// pierwszy krok czesto rozni sie od pozostalych,
	// np. do jego generacji nie wykorzystuje sie czujnikow
	// (zadanie realizowane przez klase konkretna)

	virtual bool next_step (void) = 0;
	// generuje kazdy nastepny krok ruchu
	// (zadanie realizowane przez klase konkretna)

	void re_run(void); // powrot do stanu wyjsciowego

	// Kopiuje dane z robotow do generatora
	void copy_data(std::map <ROBOT_ENUM, mp_robot*>& _robot_m);

	// Kopiuje polecenie stworzone w generatorze do robotow
	void copy_generator_command (std::map <ROBOT_ENUM, mp_robot*>& _robot_m);

  class MP_error {  // Klasa obslugi bledow generatora na poziomie MP
    public:
      const uint64_t error_class;
      const uint64_t mp_error;
      MP_error (uint64_t err0, uint64_t err1);
  }; // end: class MP_error

}; // end: class mp_generator

#endif /*MP_GENERATOR_H_*/
